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
     * std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                                         // transformations
     * std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
     * 
     * // parameters gotten
     *  int wait_timeout;;
     *  std::string scan_topic;
     *  float cluster_distance;
     *  bool smart_cluster;
     *  int min_points;
     *  float min_distance
     *  float max_radius;
     *  float max_mse;
     */

    /* CLASS CONSTRUCTOR */
    CircleDetector::CircleDetector(const rclcpp::NodeOptions &options)
    : Node("CircleDetector", options), scan_received_(false) {

        RCLCPP_INFO(this->get_logger(), "CircleDetector node started.");

        declare_get_parameters();
        
        // SERVER for /detect_cricles
        service_ = this->create_service<srv::DetectCircles>(
            "detect_circles", std::bind(&CircleDetector::service_callback, this, _1, _2)
        );
        
        // SUBSCRIBER to scan topic
        scan_sub_ = this->create_subscription<LaserScan>(
            scan_topic, 10, 
            std::bind(&CircleDetector::scan_callback, this, _1)
        );

        // TF2 listener and buffer
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "CircleDetector initialized and waiting for requests.");
    }

    /* PARAMETERS */
    void CircleDetector::declare_get_parameters(){
        // parameter declaration
        this->declare_parameter<int>("wait_timeout", 5);
        this->declare_parameter<std::string>("scan_topic", "/scan");
        this->declare_parameter<float>("cluster_distance", 0.3);
        this->declare_parameter<bool>("smart_cluster", false);
        this->declare_parameter<int>("min_points", 3);
        this->declare_parameter<float>("min_distance", 0.0f);
        this->declare_parameter<float>("max_radius", 0.5);
        this->declare_parameter<float>("max_mse", 0.04);

        // obtain values
        wait_timeout = this->get_parameter("wait_timeout").as_int();
        scan_topic = this->get_parameter("scan_topic").as_string();
        cluster_distance = static_cast<float>(this->get_parameter("cluster_distance").as_double());
        smart_cluster = this->get_parameter("smart_cluster").as_bool();
        min_distance = static_cast<float>(this->get_parameter("min_distance").as_double());
        min_points = this->get_parameter("min_points").as_int();
        max_radius = static_cast<float>(this->get_parameter("max_radius").as_double());
        max_mse = static_cast<float>(this->get_parameter("max_mse").as_double());

        // show on screen
        RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
        RCLCPP_INFO(this->get_logger(), "  wait_timeout: %d", wait_timeout);
        RCLCPP_INFO(this->get_logger(), "  scan_topic: %s", scan_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  cluster_distance: %.3f", cluster_distance);
        RCLCPP_INFO(this->get_logger(), "  smart_cluster? %s", smart_cluster ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  min_points: %d", min_points);
        RCLCPP_INFO(this->get_logger(), "  min_distance: %.3f", min_distance);
        RCLCPP_INFO(this->get_logger(), "  max_radius: %.3f", max_radius);
        RCLCPP_INFO(this->get_logger(), "  max_mse: %.4f", max_mse);
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
        std::vector<utils::Cluster> clusters = utils::process_scan(*msg, smart_cluster, cluster_distance,   // clustering
                                                                    min_points, min_distance,               // filtering
                                                                    max_radius, max_mse);                   // circle fitting

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