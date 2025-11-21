/* Author: Capuzzo Daniele */

#ifndef CIRCLE_DETECTOR_HPP_
#define CIRCLE_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>                                    // component managements

#include "sensor_msgs/msg/laser_scan.hpp"                       // lidar data
#include "geometry_msgs/msg/pose_array.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>

#include "group28_assignament_1/utilities.hpp"                  // UTILITIES (custom)

#include "group28_assignament_1/srv/detect_circles.hpp"         // custom messages

#include <tf2_ros/buffer.h>                                     // transformations
#include <tf2_ros/transform_listener.h>

#include <mutex>                                                // C++ ones
#include <string>
#include <vector>
#include <condition_variable>

namespace detection{

    // TODO: CHANGE COMMMENT HERE
    /** @brief ROS2 composable node which provides a structured
     *         way to detect tables/circles using 2D LiDAR data.
     * 
     * This node constitutes the /detect_circles SERVER, which upon
     * receiving a request (where a target frame is indicated)
     * temporarily subscribes to the /scan topic, offloads circle detection 
     * to an external function and returns the tables estimated center poses
     * in the given frame.
     *  
     * This class follows the subscription-on-demand pattern (for /scan)
     * to avoid race conditions and be able to run even on 1 thread.
     */
    class CircleDetector : public rclcpp::Node{
    
    public:
        
        /** @brief class constructor
         * Initializes member variables, handles the instantiation of /detect_tags server
         * as well as the subscription to /scan and /tf topics.
         */
        explicit CircleDetector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    private:
        /* MEMBER VARIABLES */
        std::vector<cv::Point2f> detections;                                                // detections storage
        rclcpp::Service<group28_assignament_1::srv::DetectCircles>::SharedPtr service_;     // service server
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;             // scan subscription
        sensor_msgs::msg::LaserScan::SharedPtr last_scan_;                                  // latest scan
        bool scan_received_;                                                                // flag for scan reception

        std::mutex scan_mutex_;                                                             // thread synchronization                      
        std::condition_variable scan_cv_;
        int wait_timeout;                                       

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                                        // transformations
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        
        /* CALLBACKS */

        /** @brief callback for /detect_circles, which starts the 
         *         circle detection pipeline and returns a response.
         *  
         * Called upon receiving a detect_circles request, saves the requested r.f
         * then ensures that a scan has been received, processes it to detect circles
         */
        void service_callback(const std::shared_ptr<group28_assignament_1::srv::DetectCircles::Request> req,
                              std::shared_ptr<group28_assignament_1::srv::DetectCircles::Response> res);
        
        /**
         * @brief callback that receives a single LiDAR scan.
         *
         * Saves the received scan in a member variable protected by mutex
         * to make the code thread-safe. 
         */                     
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        /**
         * @brief clears previous detections and resets node.
         */
        void reset();

    }; // class CircleDetector

    /* NON-MEMBER FUNCTIONS */
    // TODO: WRITE COMMENTS HERE
    std::vector<cv::Point2f> transform_centers(const std::vector<utils::Cluster>& clusters, const geometry_msgs::msg::TransformStamped& transform);

    geometry_msgs::msg::Pose point2pose(const cv::Point2f& point);

} // namespace detection

#endif  // CIRCLE_DETECTOR_HPP_
