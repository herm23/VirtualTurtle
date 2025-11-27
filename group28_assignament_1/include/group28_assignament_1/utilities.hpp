#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <cmath>
#include <iostream>

namespace utils {

    /*************/
    /* CONSTANTS */
    /*************/

    // standard image size in x, y and scale (pixels per meter)
    constexpr int SIZE_X = 800;
    constexpr int SIZE_Y = 800;
    constexpr float SCALE = 100.0f;

    // tolerances
    constexpr float TOL = 1e-6f;
    constexpr float REJECT_THRESH = 4.0f;

    // standard hough settings
    constexpr int MIN_HOUGH_VOTES = 15;

    /**********/
    /* BASICS */
    /**********/

    /* Custom data structore representing a cluster
     * namely a set of points, with a centroid.
     * Type represents the classification assigned with PCA.
     */
    struct Cluster {
        std::vector<cv::Point2f> points;
        cv::Point2f centroid;       // this is the centre if the cluster is a circle
        char type='u';              //'c'=circle, 'l'=line, 'u'=unassigned 
        float radius=0.0f;
    };

    /** @brief operator<< overloaded for cluster struct to show
     *         it easily on standard output.
     */
    std::ostream& operator<<(std::ostream& os, const Cluster& cls) {
        os << "Cluster{centroid=(" << cls.centroid.x <<", " << cls.centroid.y << ")}";
        return os;
    }


    /** @brief Function that converts data from laserscans
     *         into a set of 2D points in a cartesian frame
     * @param scan Input laser scan
     * @param points Output vector of 2D points
     */
    void lidar2pts(const sensor_msgs::msg::LaserScan& scan, std::vector<cv::Point2f>& points);

    /** @brief Converts a cluster into an Nx2 matrix of points
     *         to work with opencv linalg routines. The matrix is NOT a binary image.
     * @param cls Input cluster
     * @param mat Output matrix of points.
     */
    void cluster2mat(const Cluster& cls, cv::Mat& mat);


    /**************/
    /* CLUSTERING */
    /**************/
    
    /** @brief Performs simple, distance based clustering
     *         using O(N^2) cv::partition on a point array.
     * @param points Input vector of 2D points
     * @param threshold Distance threshold to cluster points
     * @return Vector of clusters
     */
    std::vector<Cluster> cluster_points(const std::vector<cv::Point2f>& points, const float threshold=0.05f);

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
    std::vector<Cluster> smart_cluster_points(const std::vector<cv::Point2f>& points, float threshold=0.05f);


    /** @brief Computes the centroid of each cluster
     * @param clusters Input/output vector of clusters (data modified in place)
     */
    void compute_centroids(std::vector<Cluster>& clusters);

    /** @brief Refines clusters removing noise
     *         and small clusters, or those whose centroid
     *         is too close (inside the robot)
     * @param clusters Input/output vector of clusters
     * @param min_points Minimum number of points to consider a valid cluster
     * @param min_distance Minimum distance of centroid from origin
     */
    void refine_clusters(std::vector<Cluster>& clusters, size_t min_points=3, float min_distance=0.1f);

    /** @brief reject clusters with more than min_points, containing lines
     *         discovered thanks to the hough transform.
     *  @param clusters a vector of clusters
     *  @param min_points the minimum number of points to consider as line
     */
    void discard_lines(std::vector<Cluster>& clusters, size_t min_points=15);

    /** @brief checks whether a cluster contains lines or not
     *         using the hough (line) transform.
     *  @param clusters a vector of clusters
     *  @param min_points the minimum number of points to consider as line
     */
    void is_line(Cluster& cls);


    /** @brief Performs circle detection by fitting a circle 
     *         or robust ellipse (>5 points) rejecting data if bad fit
     * @param clusters Input/output vector of clusters
     * @param max_radius Maximum radius to consider a cluster as a circle
     * @param max_residual Maximum residual (MAE) to consider a cluster as a circle
     */
    void detect_circles(std::vector<Cluster>& clusters, float max_radius=1.0f, float max_residual=0.02f);

    /** @brief Fits a circle to the cluster points using Kasa
     *         algorithm and opencv linear algebra singular value decomposition.
     *  @param cls Input cluster
     *  @param center Output center of the fitted circle
     *  @param radius Output radius of the fitted circle
     */
    void algebraic_circle_fit(Cluster& cls, cv::Point2f& center, float& radius);

    /** @brief Chains the pipeline together
     *         to fully process a single scan
     * @param scan Input laser scan
     * @param cluster_threshold Distance threshold to cluster points
     * @param min_points Minimum number of points to consider a valid cluster
     * @param min_distance Minimum distance of centroid from origin
     * @param max_radius Maximum radius to consider a cluster as a circle
     * @param max_residual Maximum residual (MAE) to consider a cluster as a
     * @return Vector of processed clusters
     */
    std::vector<Cluster> process_scan(const sensor_msgs::msg::LaserScan& scan,
                                      bool smart_clustering=false,
                                      float cluster_threshold=0.3f,
                                      size_t min_points=3,
                                      float min_distance=0.1f,
                                      float max_radius=1.0f,
                                      float max_residual=0.05f);



    /* HELPER FUNCTIONS */

    /** @brief Converts cluster vector into a colored or binary image
     * @param clusters Input vector of clusters
     * @param output Output image where to draw the clusters
     * @param image_size Size of the output image (image_size x image_size)
     * @param color boolean indicating whether to give a colored or binary image
     * @param scale Scaling factor to convert from world to image coordinates
     */
    void clusters2image(const std::vector<Cluster>& clusters, cv::Mat& output, int sizex=SIZE_X, int sizey=SIZE_Y, bool color=false, float scale=SCALE);

    /** @brief quickly rasterizes the cluster into a binary image
     *  @param cls constant input cluster reference
     *  @param img output binary image reference
     *  @param bbox constant bounding box enclosing the cluster
     *  @param scale pixel density per meter
     */
    void rasterize_cluster(const Cluster& cls, cv::Mat& img, const cv::Rect& bbox, int scale=SCALE);

}  // namespace utils

#endif  // UTILITIES_HPP
