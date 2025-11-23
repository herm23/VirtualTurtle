/* Author: Capuzzo Daniele */

#include "group28_assignament_1/circle_detector.hpp"

using namespace std::placeholders; // for _1, _2, ...
using namespace group28_assignament_1;
using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;

namespace detection{

    /** MEMBER VARIABLES RECAP 
     * rclcpp::Service<group28_assignament_1::srv::DetectCircles>::SharedPtr service_;      // service server
     * rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;                                // scan subscription
     * LaserScan::SharedPtr last_scan_;                                                     // latest scan
     * bool scan_received_;                                                                 // flag for scan reception
     * std::mutex scan_mutex_;                                                              // thread synchronization
     * std::condition_variable scan_cv_;
     * int wait_timeout;                                 
     * std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                                         // transformations
     * std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
     */

    /* CLASS CONSTRUCTOR */
    CircleDetector::CircleDetector(const rclcpp::NodeOptions &options)
    : Node("CircleDetector", options), scan_received_(false), wait_timeout(5) {

        RCLCPP_INFO(this->get_logger(), "CircleDetector node started.");
        
        // SERVER for /detect_cricles
        service_ = this->create_service<srv::DetectCircles>(
            "detect_circles", std::bind(&CircleDetector::service_callback, this, _1, _2)
        );
        
        // SUBSCRIBER to scan topic
        scan_sub_ = this->create_subscription<LaserScan>(
            "/scan", 10, 
            std::bind(&CircleDetector::scan_callback, this, _1)
        );

        // TF2 listener and buffer
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "CircleDetector initialized and waiting for requests.");
    }

    /* CALLBACKS */
    void CircleDetector::service_callback(const std::shared_ptr<srv::DetectCircles::Request> req,
                                          std::shared_ptr<srv::DetectCircles::Response> res){
        
        RCLCPP_INFO(this->get_logger(), "REQUEST received for frame: %s", req->target_frame.c_str());
        LaserScan::SharedPtr scan; // local copy to work on

        // wait for the scan with timeout
        std::unique_lock<std::mutex> lock(scan_mutex_);
        if(!scan_cv_.wait_for(lock, std::chrono::seconds(wait_timeout), [this]{ return scan_received_; })){
            RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for scan.");
            res->success = false;
            return;
        }
        scan = std::make_shared<LaserScan>(*last_scan_);    // store deep-copy to avoid re-locking
        scan_received_ = false;                             // reset flag
        lock.unlock();
        
        RCLCPP_DEBUG(this->get_logger(), "GOT THE SCAN NEEDED");
        
        // start the detection pipeline
        // detections stored as local variable to ensure full isolation
        auto detections = this->detect_circles(scan, req->target_frame);

        // produce the response
        res->circles.header.frame_id = req->target_frame;
        res->circles.header.stamp = scan->header.stamp;
        for (const auto & det : detections)
            res->circles.poses.push_back(point2pose(det));       // point2f -> pose

        res->success = true;
    }

    // thread-safe storage of the latest scan
    // processing is performed in the service callback
    void CircleDetector::scan_callback(const LaserScan::SharedPtr msg){
        RCLCPP_DEBUG(this->get_logger(), "SCAN CALLBACK CALLED...");
        {   // protects acces to scan
            std::lock_guard<std::mutex> lock(scan_mutex_);
            last_scan_ = msg;
            scan_received_ = true;
        }
        scan_cv_.notify_all(); // alert waiting threads (scan received)
    }

    
    /* HELPERS*/
    std::vector<cv::Point2f> CircleDetector::detect_circles(const LaserScan::SharedPtr msg, std::string target_frame) {

        // create a vector to store return values
        std::vector<cv::Point2f> results;
        // compute clusters and stats
        std::vector<utils::Cluster> clusters = utils::process_scan(*msg, true, 0.3f, 3, 0, 2.0f, 1.8f, 0.05f);

        // show the clusters here
        // cv::Mat latest_frame;
        // utils::visualize_clusters(clusters, latest_frame);
        // cv::imshow("Clusters", latest_frame);
        // cv::waitKey(1);

        // obtain the transform to the target frame
        TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(target_frame, msg->header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,  // throttle to once per 5 seconds
                "Could not get transform: %s", ex.what()
            );
            return {};
        }

        // transform all the centers (already discards non circles)
        std::vector<cv::Point2f> transformed_centers = transform_centers(clusters, transform);

        // save transformed centers
        for (const auto & center : transformed_centers) {
            results.push_back(center);
            RCLCPP_INFO(this->get_logger(), "New table detected at (%.2f, %.2f)", center.x, center.y);
        }

        return results;
    }


    /* NON-MEMBER FUNCTIONS*/
    std::vector<cv::Point2f> transform_centers(const std::vector<utils::Cluster>& clusters, const TransformStamped& transform){
        std::vector<cv::Point2f> transformed_centers;
        for (const auto& cluster : clusters) {
            if (cluster.type == 'c') {  // only circles
                // create point in source frame
                PointStamped point_in, point_out;
                point_in.point.x = cluster.centroid.x;
                point_in.point.y = cluster.centroid.y;
                point_in.point.z = 0.0;

                // transform to base_link
                tf2::doTransform(point_in, point_out, transform);
                transformed_centers.emplace_back(point_out.point.x, point_out.point.y);
            }
        }   
        return transformed_centers;
    }

    Pose point2pose(const cv::Point2f& point) {
        Pose pose;
        pose.position.x = point.x;  // 2d point
        pose.position.y = point.y;
        pose.position.z = 0.0;
        
        pose.orientation.x = 0.0;   // no orientation data
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        return pose;
    }
}

// register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(detection::CircleDetector)