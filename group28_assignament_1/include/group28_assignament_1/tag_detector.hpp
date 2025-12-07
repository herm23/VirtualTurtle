/* Author: Capuzzo Daniele */
#ifndef TAG_DETECTOR_HPP_
#define TAG_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>                                    // component managements
#include "group28_assignament_1/msg/tag_detection.hpp"          // custom messages
#include "group28_assignament_1/srv/detect_tags.hpp"
#include <tf2_ros/buffer.h>                                     // trasnformations
#include <tf2_ros/transform_listener.h>
#include <memory>                                               // C++ ones
#include <string>
#include <vector>

namespace detection{

    /* This class represents the node responsible for direct communication
     * with the tweaked apriltag_ros package for apriltag detection
     *
     * The pipeline prescribes returning the apriltags poses in the 
     * specified reference frame, upon receiving a /detect_tags request.
     */
    class TagDetector : public rclcpp::Node{
    
    public:
        /* class constructor which handles tf subscriptions as well 
         * as service callbacks (more detail on specific functions)
         */
        explicit TagDetector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    private:
        // member variables
        rclcpp::Service<group28_assignament_1::srv::DetectTags>::SharedPtr service_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        /* Upon receiving a request the node uses the /tf topic to
         * look for the apriltag center and returns it in
         * the desired reference frame
         */
        void service_callback(const std::shared_ptr<group28_assignament_1::srv::DetectTags::Request> req,
                              std::shared_ptr<group28_assignament_1::srv::DetectTags::Response> res);
        
        /* This function handles apriltag detection
         * as well as transformation of points
         */
        void process_tag( const uint32_t id, const std::string target_rf, 
                          std::shared_ptr<group28_assignament_1::srv::DetectTags::Response> res);
    };
}

#endif  // TAG_DETECTOR_HPP_
