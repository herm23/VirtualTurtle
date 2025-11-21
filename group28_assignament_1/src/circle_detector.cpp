// TODO: USE CONDITION VARIABLES AND MULTI-THREADED EXECUTOR FOR EFFICIENCY

/* Author: Capuzzo Daniele */

#include "group28_assignament_1/circle_detector.hpp"

using namespace std::placeholders; // for _1, _2, ...
using namespace group28_assignament_1;

namespace detection{

    /** MEMBER VARIABLES RECAP 
     * std::vector<cv::Point2f> detections;                                                 // detections storage
     * rclcpp::Service<group28_assignament_1::srv::DetectCircles>::SharedPtr service_;      // service server
     * rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;              // scan subscription
     * sensor_msgs::msg::LaserScan::SharedPtr last_scan_;                                   // latest scan
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
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
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
        sensor_msgs::msg::LaserScan::SharedPtr scan; // local copy to work on

        // wait for the scan with timeout
        std::unique_lock<std::mutex> lock(scan_mutex_);
        if(!scan_cv_.wait_for(lock, std::chrono::seconds(wait_timeout), [this]{ return scan_received_; })){
            RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for scan.");
            res->success = false;
            return;
        }
        scan = last_scan_;          // store copy to avoid re-locking
        scan_received_ = false;     // reset flag
        lock.unlock();
        
        RCLCPP_INFO(this->get_logger(), "GOT THE SCAN NEEDED");
        
        // produce and send response
        res->success = true;

        // TODO: process the scan
        reset();

    }

    // thread-safe storage of the latest scan
    // processing is performed in the service callback
    void CircleDetector::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "SCAN CALLBACK CALLED...");
        {   // protects acces to scan
            std::lock_guard<std::mutex> lock(scan_mutex_);
            last_scan_ = msg;
            scan_received_ = true;
        }
        scan_cv_.notify_all(); // alert waiting threads (scan received)
    }

    
    /* HELPERS*/
    void CircleDetector::reset(){
        detections.clear();
        RCLCPP_INFO(this->get_logger(), "Node reset completed.");
    }

    /* NON-MEMBER FUNCTIONS*/
    // TODO: CHECK AND MODIFY
    std::vector<cv::Point2f> transform_centers(const std::vector<utils::Cluster>& clusters, const geometry_msgs::msg::TransformStamped& transform){
        std::vector<cv::Point2f> transformed_centers;
        for (const auto& cluster : clusters) {
            if (cluster.type == 'c') {  // only circles
                // create point in source frame
                geometry_msgs::msg::PointStamped point_in, point_out;
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

    geometry_msgs::msg::Pose point2pose(const cv::Point2f& point) {
        geometry_msgs::msg::Pose pose;
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