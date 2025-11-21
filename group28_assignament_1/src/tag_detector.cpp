/* Author: Capuzzo Daniele */

#include "group28_assignament_1/tag_detector.hpp"

namespace detection {

    // class constructor
    TagDetector::TagDetector(const rclcpp::NodeOptions& options) : Node("TagDetector", options) {
        
        RCLCPP_INFO(this->get_logger(), "TagDetector started");
        // initialize service SERVER for detection
        service_ = this->create_service<group28_assignament_1::srv::DetectTags>(
            "detect_tags", std::bind(&TagDetector::service_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        // init TF2 listener and buffer
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "TagDetector ready for detection");

    }

    // callback for /detect_tags service
    void TagDetector::service_callback( const std::shared_ptr<group28_assignament_1::srv::DetectTags::Request> req,
                           std::shared_ptr<group28_assignament_1::srv::DetectTags::Response> res){

        if(req->ids.empty()){
            RCLCPP_WARN(this->get_logger(), "Provide at least one apriltag id");
            res->success = false;
        }
        if(req->target_frame.compare("map") == 0){
            RCLCPP_WARN(
                this->get_logger(), 
                "issue with map as target frame: AMCL and tf use different timestamps. If nothing is received, try again using odom. Otherwise the transform has been found"
            );
        }

        // offload work to process_tag
        // note: cannot use threads here
        for(uint32_t id : req->ids)
            process_tag(id, req->target_frame, res);
        
        RCLCPP_INFO(this->get_logger(), "Response sent");
    }

    void TagDetector::process_tag( const uint32_t id, const std::string target_rf, 
                      std::shared_ptr<group28_assignament_1::srv::DetectTags::Response> res) {

        try {
            // Look up transform
            auto t = tf_buffer_->lookupTransform(
                target_rf,      
                "tag36h11:" + std::to_string(id),    
                tf2::TimePointZero 
            );
            RCLCPP_INFO(this->get_logger(), "\t tag36h11:%d found", id);
            group28_assignament_1::msg::TagDetection detection;
            detection.id = id;
            detection.name = "tag36h11:" + std::to_string(id);
            detection.pose.header = t.header;
            
            // the tag center is always (0,0,0) in its r.f
            // so just use translation/rotation
            detection.pose.pose.position.x = t.transform.translation.x;
            detection.pose.pose.position.y = t.transform.translation.y;
            detection.pose.pose.position.z = t.transform.translation.z;
            detection.pose.pose.orientation = t.transform.rotation;
            
            res->tags.push_back(detection);
            res->success = true;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Tag %d NOT found: %s", id, ex.what());
        }
    }

} // namespace detection

// register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(detection::TagDetector)