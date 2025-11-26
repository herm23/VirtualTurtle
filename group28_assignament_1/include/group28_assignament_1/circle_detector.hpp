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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mutex>                                                // C++ ones
#include <string>
#include <vector>
#include <condition_variable>

namespace detection{

    /** @brief ROS2 composable node which provides a structured
     *         way to detect tables/circles using 2D LiDAR data.
     * 
     * This node constitutes the /detect_circles SERVER, and simultaneously subscribes to /scan.
     * Upon receiving a request, the node waits for a LiDAR scan to be available,
     * processes it to detect circles and returns their estimated poses in the requested frame.
     *  
     * The request indicates the target frame for the detected circles poses
     * and the response contains an array of geometry_msgs/Pose messages of
     * the detected circles centers in the correct frame.
     * A success field is included to indicate whether the detection was successful.
     *  
     * NOTE: this node needs to be run in a multithreaded executor
     *       to handle /scan and /detect_circles requests simultaneously.
     */
    class CircleDetector : public rclcpp::Node{
    
    public:
        
        /** @brief class constructor
         *  @param options: node options
         * Initializes member variables, handles the instantiation of /detect_tags server
         * as well as the subscription to /scan and /tf topics.
         */
        explicit CircleDetector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    private:
        /* MEMBER VARIABLES */
        rclcpp::Service<group28_assignament_1::srv::DetectCircles>::SharedPtr service_;     // service server
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;             // scan subscription
        sensor_msgs::msg::LaserScan::SharedPtr last_scan_;                                  // latest scan
        bool scan_received_;                                                                // flag for scan reception

        std::mutex scan_mutex_;                                                             // thread synchronization                      
        std::condition_variable scan_cv_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                                        // transformations
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // parameters gotten
        int wait_timeout;
        std::string scan_topic;
        float cluster_distance;
        bool smart_cluster;
        int min_points;
        float min_distance;
        float max_radius;
        float max_mae;


        /* PARAMETER SETTING */

        /** @brief function used to declare and set parameters
         *         to keep the node more general and work with different configs
         */
        void declare_get_parameters();

        
        /* CALLBACKS */

        /** @brief callback for /detect_circles, which starts the 
         *         circle detection pipeline and returns a response.
         *  @param req: pointer to the received request
         *  @param res: pointer to the response to be sent
         * Called upon receiving a detect_circles request, saves the requested r.f
         * then ensures that a scan has been received, processes it to detect circles
         */
        void service_callback(const std::shared_ptr<group28_assignament_1::srv::DetectCircles::Request> req,
                              std::shared_ptr<group28_assignament_1::srv::DetectCircles::Response> res);
        
        /** @brief callback that receives a single LiDAR scan.
         *  @param msg: pointer to the received LaserScan message
         *
         * Saves the received scan in a member variable protected by mutex
         * to make the code thread-safe. Then notifies waiting threads that a scan is available.
         */                     
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        /** @brief processes a given LaserScan to detect circles
         *  @param msg: pointer to the LaserScan message to be processed
         *  @param target_frame: reference frame in which to express the detected circles poses
         *  Populates the detections member variable with the detected circles centers in the target frame.
         */
        std::vector<cv::Point2f> detect_circles(const sensor_msgs::msg::LaserScan::SharedPtr msg, std::string target_frame);

    }; // class CircleDetector

    /* NON-MEMBER FUNCTIONS */
    /** @brief transforms only circle centers using the provided transformation
     *  @param clusters: vector of clusters representing detected circles
     *  @param transform: transformation to be applied to the centers
     *  @return vector of transformed circle centers
     */
    std::vector<cv::Point2f> transform_centers(const std::vector<utils::Cluster>& clusters, const geometry_msgs::msg::TransformStamped& transform);

    /** @brief converts a 2D point to a geometry_msgs::Pose message
     *  @param point: 2D point to be converted
     *  @return geometry_msgs::msg::Pose message representing the point
     */
    geometry_msgs::msg::Pose point2pose(const cv::Point2f& point);

} // namespace detection

#endif  // CIRCLE_DETECTOR_HPP_
