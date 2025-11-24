#include "group28_assignament_1/utilities.hpp"

namespace utils {

    /** @brief Function that converts data from laserscans
     *         into a set of 2D points in a cartesian frame
     * @param scan Input laser scan
     * @param points Output vector of 2D points
     */
    void lidar2pts(const sensor_msgs::msg::LaserScan& scan, std::vector<cv::Point2f>& points){
        // create vector of points and reserve memory
        // to avoid unnecessary resizings
        points.clear();
        points.reserve(scan.ranges.size());

        // proceed along the scan and convert all points
        float angle = scan.angle_min;
        for (float r : scan.ranges) {
            if (std::isfinite(r)) {  // avoids issues with NaN
                points.emplace_back(r * std::cos(angle), r * std::sin(angle));
            }
            angle += scan.angle_increment;
        }
    }

    /** @brief Converts a cluster into an Nx2 matrix of points
     *         to work with opencv linalg routines. The matrix is NOT a binary image.
     * @param cls Input cluster
     * @param mat Output matrix of points.
     */
    void cluster2mat(const Cluster& cls, cv::Mat& mat){
        if(cls.points.empty()) {mat = cv::Mat(); return;}

        mat.create(cls.points.size(), 2, CV_32F);   // if matrix is already of this type
                                                    //just reuses the memory, else creates
        
        // run through matrix and add points from the cluster
        for (size_t i = 0; i < cls.points.size(); ++i) {
            const cv::Point2f& p = cls.points[i];
            float x = std::isfinite(p.x) ? p.x : 0.f;   // checks for bad data
            float y = std::isfinite(p.y) ? p.y : 0.f;

            mat.at<float>(i, 0) = x;                    // safe access
            mat.at<float>(i, 1) = y;
        }
    }
    
    /** @brief Performs simple, distance based clustering
     *         using O(N^2) cv::partition on a point array.
     * @param points Input vector of 2D points
     * @param threshold Distance threshold to cluster points
     * @return Vector of clusters
     */
    std::vector<Cluster> cluster_points(const std::vector<cv::Point2f>& points, float threshold){
        if(points.empty()) return {};

        // define a predicate to perform distance based clustering
        // points are clustered together if they are closer than threshold
        auto predicate = [threshold](const cv::Point2f& a, const cv::Point2f& b){
            return cv::norm(a - b) < threshold;
        };

        std::vector<int> labels;
        int n_clusters = cv::partition(points, labels, predicate);

        // now actually build the clusters
        std::vector<Cluster> clusters(n_clusters);
        for (size_t i = 0; i < points.size(); ++i) {
            int idx = labels[i];
            clusters[idx].points.push_back(points[i]);
        }
        return clusters;
    }

    /** @brief Performs smart, distance based clustering
     *         exploiting the ordered nature of laser scan points. 
     * 
     * Laser scans are acquired in order, hence points that are close in the vector
     * are also close in space. This function exploits this fact to perform distance based
     * agglomerative clustering in O(N) time: only consecutive points are compared.
     * 
     * @param points Input vector of 2D points
     * @param threshold Distance threshold to cluster points
     * @return Vector of clusters
     */
    std::vector<Cluster> smart_cluster_points(const std::vector<cv::Point2f>& points, float threshold){
        if(points.empty()) return {};

        std::vector<Cluster> clusters;
        // init first cluster with first point
        Cluster initial_cluster;
        initial_cluster.points.push_back(points[0]);
        clusters.push_back(initial_cluster);

        // points are ordered because of the laser scan range
        // so we only need to check distance with previous point
        for(size_t i = 1; i < points.size(); ++i){
            if(cv::norm(points[i] - points[i-1]) < threshold) { // close enough -> same cluster
                clusters.back().points.push_back(points[i]);
            } else {                                            // new cluster
                Cluster new_cluster;
                new_cluster.points.push_back(points[i]);
                clusters.push_back(std::move(new_cluster));
            }
        }

        return clusters;
    }

    /** @brief Computes centroids using smart matrix operations
     * @param clusters Input/output vector of clusters (data modified in place)
     */
    void compute_centroids(std::vector<Cluster>& clusters){
        for (auto& cluster : clusters) {
            // skip empty clusters
            if (cluster.points.empty()) { 
                cluster.centroid = cv::Point2f(0.0f, 0.0f);
                continue; 
            }

            // Convert to cv::Mat and compute mean
            cv::Mat cls_mat(cluster.points.size(), 2, CV_32F);
            cluster2mat(cluster, cls_mat);
            cv::Scalar mean = cv::mean(cls_mat);
            cluster.centroid = cv::Point2f(static_cast<float>(mean[0]), static_cast<float>(mean[1]));
        }
    }

    /** @brief Refines clusters removing noise
     *         and small clusters, or those whose centroid
     *         is too close (inside the robot)
     * @param clusters Input/output vector of clusters
     * @param min_points Minimum number of points to consider a valid cluster
     * @param min_distance Minimum distance of centroid from origin
     */
    void refine_clusters( std::vector<Cluster>& clusters, size_t min_points, float min_distance) {                            
        // remove clusters with less than min_points
        // or those whose centroid is too close
        auto iterator = std::remove_if(clusters.begin(), clusters.end(),
                        [min_points, min_distance](const Cluster& c) {
                            return c.points.size() < min_points || cv::norm(c.centroid) < min_distance;
                        });
        clusters.erase(iterator, clusters.end());
    }

    void discard_lines(std::vector<Cluster>& clusters, size_t min_points){
        for(auto cls: clusters){
            if(cls.points.size() > min_points) is_line(cls);
        }
    }

    void is_line(Cluster& cls){
        // first we need to determine a bounding
        // box containing the whole cluster
        float minx, maxx, miny, maxy;
        for (const auto& p : cls.points) {
            minx = std::min(minx, p.x);
            maxx = std::max(maxx, p.x);
            miny = std::min(miny, p.y);
            maxy = std::max(maxy, p.y);
        }

        float width = maxx - minx;
        float height = maxy - miny;

        // build an image for the cluster
        cv::Mat img;
        clusters2image({cls}, img, width+10, height+10, false, 100.0f);

        // now do the hough transform to detect lines
        std::vector<cv::Point2f> lines; 
        cv::HoughLines(img, lines, 1, CV_PI / 180, 20);

        // if at least one line was found, classify as line
        if(!lines.empty())
            cls.type='l';
    }


    /** @brief Performs circle detection by fitting a circle 
     *         or robust ellipse (>5 points) rejecting data if bad fit
     * @param clusters Input/output vector of clusters
     * @param max_radius Maximum radius to consider a cluster as a circle
     * @param max_residual Maximum residual (MAE) to consider a cluster as a circle
     */
    void detect_circles(std::vector<Cluster>& clusters, float max_radius, float max_residual){

        for( auto& cls: clusters ){
            int n = cls.points.size();
            if(n==0 || cls.type=='l') continue;     // skip pre-rejected lines

            cv::Point2f center;                     // placeholders
            float radius=0.f;
            if(n < 3)
                cv::minEnclosingCircle(cls.points, center, radius); // fallback fit circle
            else
                algebraic_circle_fit(cls, center, radius);          // needs 5 points
                
            // we have circle candidates that still need to be verified
            if(radius > max_radius){ cls.type='l'; continue; } // radius too high -> line

            // compute residual (MAE)
            float mae = 0.0f;
            for (const auto& p : cls.points)
                mae += std::abs(cv::norm(p - center) - radius);

            mae /= cls.points.size();

            // bad fit --> line
            if (mae > max_residual) { cls.type = 'l'; continue; }

            // good fit --> circle
            cls.centroid = center;
            cls.radius = radius;
            cls.type = 'c';
        }
    }

    /** @brief Fits a circle to the cluster points using Kasa
     *         algorithm and opencv linear algebra singular value decomposition.
     *  @param cls Input cluster
     *  @param center Output center of the fitted circle
     *  @param radius Output radius of the fitted circle
     */
    void algebraic_circle_fit(Cluster& cls, cv::Point2f& center, float& radius) {
        int n = cls.points.size();        
        // matrices for the linear system: A * [a, b, c]'T = B
        // x^2 + y^2 + a*x + b*y + c = 0
        cv::Mat A(n, 3, CV_64F);
        cv::Mat B(n, 1, CV_64F);

        for (int i = 0; i < n; i++) {
            double x = cls.points[i].x;
            double y = cls.points[i].y;
            A.at<double>(i, 0) = x;
            A.at<double>(i, 1) = y;
            A.at<double>(i, 2) = 1.0;
            
            B.at<double>(i, 0) = -(x*x + y*y);
        }
        // solve using SVD and cv::solve()
        cv::Mat X;
        cv::solve(A, B, X, cv::DECOMP_SVD);
        // get coefficients
        double a = X.at<double>(0, 0);
        double b = X.at<double>(1, 0);
        double c = X.at<double>(2, 0);
        // coeff to circle params
        center.x = -a / 2.0;
        center.y = -b / 2.0;
        radius = std::sqrt(a*a/4.0 + b*b/4.0 - c);
    }


    /** @brief Chains the pipeline together to fully process a single scan
     * @param scan Input laser scan
     * @param smart_clustering Whether to use smart clustering or not (requires euclidian-ordered array)
     * @param cluster_threshold Distance threshold to cluster points
     * @param min_points Minimum number of points to consider a valid cluster
     * @param min_distance Minimum distance of centroid from origin
     * @param max_radius Maximum radius to consider a cluster as a circle
     * @param max_residual Maximum residual (MAE) to consider a cluster as a
     * @return Vector of processed clusters
     */
    std::vector<Cluster> process_scan(const sensor_msgs::msg::LaserScan& scan, bool smart_clustering,
                                      float cluster_threshold, size_t min_points, float min_distance, 
                                      float max_radius, float max_residual){
        // 1-convert scan to points
        std::vector<cv::Point2f> points;
        lidar2pts(scan, points);

        // 2-cluster points
        std::vector<Cluster> clusters;
        if(smart_clustering)
            clusters = smart_cluster_points(points, cluster_threshold);
        else
            clusters = cluster_points(points, cluster_threshold);

        // 2-compute centroids
        compute_centroids(clusters);

        // 4-refine clusters (remove noise and invalid clusters)
        // only if the minimum number of points is not 0
        if( min_points!=0 )
            refine_clusters(clusters, min_points, min_distance);

        // 5-discard most lines
        discard_lines(clusters, 15);

        // 6-detect circles
        detect_circles(clusters, max_radius, max_residual);

        return clusters;        
    }

    /** @brief Converts cluster vector into a colored or binary image
     * @param clusters Input vector of clusters
     * @param output Output image where to draw the clusters
     * @param sizex Size of the output image x axis (sizex x sizey)
     * @param sizey Size of the output image y axis (sizex x sizey)
     * @param color boolean indicating whether to give a colored or binary image
     * @param scale Scaling factor to convert from world to image coordinates
     */
    void clusters2image(const std::vector<Cluster>& clusters, cv::Mat& output, int sizex, int sizey, bool color, float scale) {
        if(clusters.empty() || sizex<=0 || sizey<=0 ||scale <=0) return;

        if(color)
            output = cv::Mat(sizex, sizey, CV_8UC3, cv::Scalar(255, 255, 255));
        else
            output = cv::Mat(sizex, sizey, CV_8UC1, cv::Scalar(0));

        cv::Point2f center(sizex / 2.0f, sizey / 2.0f);
        cv::RNG rng(12345);
        for (const auto& cls : clusters) {
            if (cls.points.empty()) continue;

            // get color randomly for cluster or white if binary image
            cv::Scalar col = color ? cv::Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255)) :
                                     cv::Scalar(255);
                           
            // Draw all points in the cluster
            for (const auto& p : cls.points) {
                // Scale and flip y to image coordinates
                cv::Point pt(static_cast<int>(center.x + p.x * scale),
                            static_cast<int>(center.y - p.y * scale));
                if (pt.x >= 0 && pt.x < sizex && pt.y >= 0 && pt.y < sizey)
                    cv::circle(output, pt, 2, col, cv::FILLED);
            }

            // draw a circle wherever necessary to show detection
            cv::Point cpt(static_cast<int>(center.x + cls.centroid.x * scale),
                        static_cast<int>(center.y - cls.centroid.y * scale));
            if (cls.type == 'c' && color)
                cv::circle(output, cpt, static_cast<int>(cls.radius * scale), col, 2);
        }
    }

    /* NOTE: to visualize the clusters use:
     *  cv::Mat image;
     *  clusters2image(clusters, image, 800, false, 100.0f);
     *  cv::imshow("clusters", image);
     *  cv::waitKey(1);
     */

} // namespace