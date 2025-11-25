#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include <random>
#include <array>
#include <vector>
#include <cstdint>

#include "group28_assignament_1/srv/detect_tags.hpp"
#include "group28_assignament_1/srv/detect_circles.hpp"
#include "group28_assignament_1/msg/tag_detection.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using ServiceT = nav2_msgs::srv::ManageLifecycleNodes;
using ActionT  = nav2_msgs::action::NavigateToPose;
using GoalHandleActionT = rclcpp_action::ClientGoalHandle<ActionT>;
using DetectApriltagSrv = group28_assignament_1::srv::DetectTags;
using DetectCirclesSrv = group28_assignament_1::srv::DetectCircles;
using TagDetectionMsg  = group28_assignament_1::msg::TagDetection;

// enum class InitSteps {
//     START,
//     GET_GOAL_FROM_SERVER,      // Ottenere la posizione goal da /goal_pose
//     ACTIVATE_LOCALIZATION,     // Richiamare il service di attivazione della localization
//     INIT_START_POSITION,       // Inizializzare la posizione iniziale
//     ACTIVATE_NAVIGATION,       // Richiamare il service di attivazione della navigation
//     SEND_GOAL_ACTION           // Inviare il goal tramite action NavigateToPose
// };

enum class InitSteps {
    START_NAV, //Da start a corridorio
    MANUAL_NAV, //Da corridoio a fine corridoio
    RESUME_NAV //Da fine corridoio a destinazione
};

class NavigationNode : public rclcpp::Node
{
public:
  NavigationNode() : Node("navigation_node")
  {
    // SERVICE CLIENT PER ATTIVARE LOCALIZATION
    localization_client = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_localization/manage_nodes");
    
    // PUBLISH DELLA POSIZIONE INIZIALE
    initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

    // SERVICE CLIENT PER ATTIVARE NAVIGATION
    navigation_client = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_navigation/manage_nodes");
    
    // ACTION CLIENT PER FAR PARTIRE NAVIGATION
    navigate_action_client_ = rclcpp_action::create_client<ActionT>(
      this,
      "/navigate_to_pose"   // stesso nome del comando CLI
    );

    // SERVICE CLIENT PER DETECTION TAGS (detect_tags)
    detect_apriltag_client_ = this->create_client<DetectApriltagSrv>("/detect_tags");

    // SERVICE CLIENT PER DETECTION CIRCLES (detect_circles)
    detect_circles_client_ = this->create_client<DetectCirclesSrv>("/detect_circles");

    // LIDAR SUBSCRIBER PER PUNTI EXTRA
    lidar_sensor_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", //lidar topic
      10,     
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
      {
        // activate the callback of lidar msgs each time arrive a new scan
        this->process_lidar_data_callback(msg);
        

      });

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "localization client init\n");

  }

  #pragma region 
  rclcpp::Client<ServiceT>::SharedFuture send_localization_request()
  {
    return send_client_request(true);
  }

  rclcpp::Client<ServiceT>::SharedFuture send_navigation_request()
  {
    return send_client_request(false);
  }

  //NOTE: Gestione univoca della feature per richieste asincrone
  void handle_result(rclcpp::Client<ServiceT>::SharedFuture result, std::string serviceName) {
    if (!result.valid()) {
      rclcpp::shutdown();
      return;
    }

    auto self = this->shared_from_this();

    if (rclcpp::spin_until_future_complete(self, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = result.get();
      RCLCPP_INFO(this->get_logger(),"Response: %s",response->success ? "true" : "false");      
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service %s", serviceName.c_str());
    }
  }
  #pragma endregion

  void publish_initial_pose()
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;

    // header
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";

    // pose
    msg.pose.pose.position.x = 0.0;
    msg.pose.pose.position.y = 0.0;
    msg.pose.pose.position.z = 0.0;

    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = 0.0;
    msg.pose.pose.orientation.w = 1.0;

    // covariance (36 elementi)
    std::array<double, 36> cov = {
      0.25, 0.0,  0.0,   0.0,   0.0,   0.0,
      0.0,  0.25, 0.0,   0.0,   0.0,   0.0,
      0.0,  0.0,  0.0,   0.0,   0.0,   0.0,
      0.0,  0.0,  0.0,   0.068, 0.0,   0.0,
      0.0,  0.0,  0.0,   0.0,   0.068, 0.0,
      0.0,  0.0,  0.0,   0.0,   0.0,   0.068
    };
    msg.pose.covariance = cov;

    RCLCPP_INFO(this->get_logger(), "Publishing initial pose on /initialpose");
    initialpose_pub_->publish(msg);
  }

  void wait_for_amcl_pose()
  {
    amcl_received_ = false;

    if (!amcl_sub_) {
      amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
        {
          last_amcl_pose_ = *msg;
          amcl_received_ = true;

          RCLCPP_INFO(
            this->get_logger(),
            "Received /amcl_pose: x=%.3f, y=%.3f (frame_id=%s)",
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->header.frame_id.c_str()
          );
        });
    }

    RCLCPP_INFO(this->get_logger(), "Waiting for /amcl_pose after /initialpose...");

    auto self = this->shared_from_this();
    rclcpp::Rate rate(10);  // 10 Hz

    while (rclcpp::ok() && !amcl_received_) {
      rclcpp::spin_some(self);
      rate.sleep();
    }

    if (amcl_received_) {
      RCLCPP_INFO(this->get_logger(), "Localization updated from /amcl_pose, continuing.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Stopped waiting for /amcl_pose (rclcpp::ok() == false).");
    }
  }

  void send_navigate_goal(geometry_msgs::msg::Point goal_point)
  {
    if (!navigate_action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server /navigate_to_pose not available");
      return;
    }

    #pragma region 
    // //NOTE: Definiamo posizione statica, sarà da legere dal server "/goal_pose"
    // ActionT::Goal goal;
    // goal.pose.header.frame_id = "map";
    // goal.pose.header.stamp = this->now();

    // goal.pose.pose.position.x = 12.5;
    // goal.pose.pose.position.y = -2.6;
    // goal.pose.pose.position.z = 0.0;

    // goal.pose.pose.orientation.x = 0.0;
    // goal.pose.pose.orientation.y = 0.0;
    // goal.pose.pose.orientation.z = 0.7071;
    // goal.pose.pose.orientation.w = 0.7071;

    // RCLCPP_INFO(
    //   this->get_logger(),
    //   "Sending NavigateToPose goal: x=%.2f, y=%.2f, z=%.2f, qz=%.4f, qw=%.4f",
    //   goal.pose.pose.position.x,
    //   goal.pose.pose.position.y,
    //   goal.pose.pose.position.z,
    //   goal.pose.pose.orientation.z,
    //   goal.pose.pose.orientation.w
    // );
    #pragma endregion

    ActionT::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = this->now();

    goal.pose.pose.position.x = goal_point.x;
    goal.pose.pose.position.y = goal_point.y;
    goal.pose.pose.position.z = goal_point.z;

    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = 0.7071;
    goal.pose.pose.orientation.w = 0.7071;

    RCLCPP_INFO(
      this->get_logger(),
      "Sending NavigateToPose goal: x=%.2f, y=%.2f, z=%.2f, qz=%.4f, qw=%.4f",
      goal.pose.pose.position.x,
      goal.pose.pose.position.y,
      goal.pose.pose.position.z,
      goal.pose.pose.orientation.z,
      goal.pose.pose.orientation.w
    );

    rclcpp_action::Client<ActionT>::SendGoalOptions options;
    
    //Callback di debug
    options.goal_response_callback =
      [this](GoalHandleActionT::SharedPtr gh)
      {
        if (!gh) {
          RCLCPP_ERROR(this->get_logger(), "NavigateToPose goal was rejected by server");
        } else {
          RCLCPP_INFO(this->get_logger(), "NavigateToPose goal accepted by server");
        }
      };

    options.feedback_callback =
      [this](GoalHandleActionT::SharedPtr goal_handle,
            const std::shared_ptr<const ActionT::Feedback> feedback)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "NavigateToPose feedback: distance_remaining=%.2f",
          feedback->distance_remaining
        );

        if (isInCorridor)
        {
          RCLCPP_WARN(
            this->get_logger(),
            "IsInCorridor == true, annullo la navigazione corrente");

          navigate_action_client_->async_cancel_goal(goal_handle);
          
        }
      };

    auto self = this->shared_from_this();

    auto goal_handle_future =
      navigate_action_client_->async_send_goal(goal, options);

    if (rclcpp::spin_until_future_complete(self, goal_handle_future)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to send NavigateToPose goal");
      return;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "NavigateToPose goal_handle is null");
      return;
    }

   
    auto result_future = navigate_action_client_->async_get_result(goal_handle);

    auto result_code = rclcpp::spin_until_future_complete(self, result_future);
    if (result_code != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get result for NavigateToPose");
      return;
    }

    auto wrapped_result = result_future.get();

    switch (wrapped_result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "NavigateToPose goal SUCCEEDED");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "NavigateToPose goal ABORTED");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "NavigateToPose goal CANCELED");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "NavigateToPose goal ended with UNKNOWN result code");
        break;
    }
  }

  geometry_msgs::msg::Point call_detect_apriltags()
  {
    geometry_msgs::msg::Point midpoint;

    auto request = std::make_shared<DetectApriltagSrv::Request>();

    // ID da cercare
    std::vector<uint32_t> ids = {1, 10};
    const std::string target_frame = "odom";

    request->ids = ids;
    request->target_frame = target_frame;

    // Aspetto che il service sia disponibile
    while (!detect_apriltag_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),"Interrupted while waiting for /detect_tags service. Exiting.");
        return midpoint;
      }

      RCLCPP_INFO(this->get_logger(),"/detect_tags service not available, waiting again...");
    }

    RCLCPP_INFO(this->get_logger(),"Calling /detect_tags with %zu ids in target_frame='%s'",ids.size(), target_frame.c_str());

    auto self = this->shared_from_this();
    auto future = detect_apriltag_client_->async_send_request(request);

    auto ret = rclcpp::spin_until_future_complete(self, future);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to call /detect_tags (spin_until_future_complete)");
      return midpoint;
    }

    auto response = future.get();
    if (!response->success) {
      RCLCPP_WARN(this->get_logger(), "/detect_tags returned success=false");
      return midpoint;
    }

    std::vector<TagDetectionMsg> tags_out = response->tags;

    RCLCPP_INFO(this->get_logger(),"Received %zu tags from /detect_tags",tags_out.size());

    if (tags_out.size() < 2) {
      RCLCPP_WARN(this->get_logger(), "Less than 2 tags received, cannot compute midpoint.");
      return midpoint;
    }

    for (size_t i = 0; i < 2; ++i) {
      const auto & tag = tags_out[i];
      const auto & p = tag.pose.pose.position;
      const auto & q = tag.pose.pose.orientation;

      RCLCPP_INFO(this->get_logger(),
        "Tag[%zu] id=%u name='%s' frame_id='%s' "
        "pos=(%.3f, %.3f, %.3f) "
        "orient=(%.3f, %.3f, %.3f, %.3f)",
        i,
        tag.id,
        tag.name.c_str(),
        tag.pose.header.frame_id.c_str(),
        p.x, p.y, p.z,
        q.x, q.y, q.z, q.w
      );
    }

    const auto & p1 = tags_out[0].pose.pose.position;
    const auto & p2 = tags_out[1].pose.pose.position;

    midpoint.x = (p1.x + p2.x) / 2.0;
    midpoint.y = (p1.y + p2.y) / 2.0;
    midpoint.z = (p1.z + p2.z) / 2.0;

    RCLCPP_INFO(this->get_logger(),"Midpoint between first two tags: (%.3f, %.3f, %.3f)", midpoint.x, midpoint.y, midpoint.z);
    return midpoint;
  }

  void call_detect_circles()
  {
    auto request = std::make_shared<DetectCirclesSrv::Request>();
    const std::string target_frame = "odom";
    request->target_frame = target_frame;

    // Aspetto che il service sia disponibile
    while (!detect_circles_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),"Interrupted while waiting for /detect_circles service. Exiting.");
        return;
      }

      RCLCPP_INFO(this->get_logger(),"/detect_circles service not available, waiting again...");
    }

    RCLCPP_INFO(this->get_logger(),"Calling /detect_circles with target_frame='%s'", target_frame.c_str());

    auto self = this->shared_from_this();
    auto future = detect_circles_client_->async_send_request(request);

    auto ret = rclcpp::spin_until_future_complete(self, future);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to call /detect_circles (spin_until_future_complete)");
      return;
    }

    auto response = future.get();
    if (!response->success) {
      RCLCPP_WARN(this->get_logger(), "/detect_circles returned success=false");
      return;
    }

    geometry_msgs::msg::PoseArray circles_out = response->circles;

    RCLCPP_INFO(this->get_logger(),"Received %zu circles from /detect_circles",circles_out.poses.size());

    for (size_t i = 0; i < circles_out.poses.size(); ++i) {
      const auto & pose = circles_out.poses[i];
      const auto & p = pose.position;

      RCLCPP_INFO(this->get_logger(), "Circle[%zu]: pos = (x=%.3f, y=%.3f, z=%.3f)",i, p.x, p.y, p.z);
    }

    return;
  }

  void manual_nav()
  {
      geometry_msgs::msg::Twist msg;
      msg.linear.x = 1;
      publisher_->publish(msg);
  }

  void manual_stop()
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0;
    publisher_->publish(msg);
  }

  bool getIsInCorridor()
  {
    return isInCorridor;
  }

private:
  //Class Members
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr localization_client;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr navigation_client;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  bool amcl_received_ = false;
  geometry_msgs::msg::PoseWithCovarianceStamped last_amcl_pose_;
  rclcpp_action::Client<ActionT>::SharedPtr navigate_action_client_;
  rclcpp::Client<DetectApriltagSrv>::SharedPtr detect_apriltag_client_;
  rclcpp::Client<DetectCirclesSrv>::SharedPtr detect_circles_client_;
  
  //Subscriber lidar
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sensor_subscriber;
  
  //Publisher manual navigation
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool isInCorridor = false;

  //gestione richieste 
  rclcpp::Client<ServiceT>::SharedFuture send_client_request(bool isLocalization)
  {
    auto request = std::make_shared<ServiceT::Request>();
    request->command = ServiceT::Request::STARTUP;

    auto client = isLocalization ? localization_client : navigation_client;
    std::string serviceName = isLocalization ? "/lifecycle_manager_localization/manage_nodes" : "/lifecycle_manager_navigation/manage_nodes";

    // aspetta il service
    while (!client->wait_for_service(1s)) {
      
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Interrupted by the user. Exit!");
        return rclcpp::Client<ServiceT>::SharedFuture{};
      }

      RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Service %s not available, waiting again...", serviceName.c_str());
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Sending  request to %s| command: %u",
      serviceName.c_str(),
      static_cast<unsigned>(request->command));

    auto result = client->async_send_request(request);
    return result;
  }

  bool process_lidar_data_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
      // Copio le distanze in un vettore di double, come avevi già fatto
      std::vector<double> distances(msg->ranges.begin(), msg->ranges.end());

      const double angle_min       = static_cast<double>(msg->angle_min);
      const double angle_increment = static_cast<double>(msg->angle_increment);

      // Range di angoli che ti interessa [rad]
      //Seleziono solo l'angolatura destra
      const double target_angle_min = 3.92; //5/4 pigreco
      const double target_angle_max = 5.49; //7/4 pigreco

      double sum_distances = 0.0;
      std::size_t count    = 0;

      for (std::size_t i = 0; i < distances.size(); ++i)
      {
          double distance = distances[i];

          // Ignoro valori non finiti (NaN, inf) così non sporcano la media
          if (!std::isfinite(distance))
              continue;

          double angle = angle_min + static_cast<double>(i) * angle_increment; // [rad]

          // Considero solo i punti nel range angolare richiesto
          if (angle >= target_angle_min && angle <= target_angle_max)
          {
              sum_distances += distance;
              ++count;
          }
      }

      if (count == 0)
      {
          // RCLCPP_INFO(
          //     rclcpp::get_logger("lidar_processor"),
          //     "Nessun punto nel range angolare [%.2f, %.2f] rad.",
          //     target_angle_min, target_angle_max
          // );
          return false;
      }

      double avg_distance = sum_distances / static_cast<double>(count);

      // RCLCPP_INFO(
      //     rclcpp::get_logger("lidar_processor"),
      //     "Distanza media nel range angolare [%.2f, %.2f] rad: %.3f m (su %zu punti)",
      //     target_angle_min, target_angle_max, avg_distance, count
      // );

      // Threshold per il corridoio
      const double threshold = 1.5;
      isInCorridor = avg_distance < threshold;
      RCLCPP_INFO(this->get_logger(),"IsInCorridor = %s", isInCorridor ? "true" : "false");

      // if (isInCorridor)
      //     RCLCPP_INFO( rclcpp::get_logger("lidar_processor"), "Media %.3f < %.2f -> sei dentro al corridoio", avg_distance, threshold);
      // else
      //     RCLCPP_INFO(rclcpp::get_logger("lidar_processor"),"Media %.3f >= %.2f -> sei fuori dal corridoio", avg_distance, threshold);
      

      return true;
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<NavigationNode> nav_node = std::make_shared<NavigationNode>();
  
  //rclcpp::spin(nav_node);
  
  // STEP 0
  geometry_msgs::msg::Point goal_point = nav_node->call_detect_apriltags();

  //STEP 1
  auto result_localization = nav_node->send_localization_request();
  nav_node->handle_result(result_localization, "/lifecycle_manager_localization/manage_nodes");

  //STEP 2
  nav_node->publish_initial_pose();

   //STEP 2.5 - la posa è stata recepità ?
  nav_node->wait_for_amcl_pose();

  //STEP 3
  auto result_nav = nav_node->send_navigation_request();
  nav_node->handle_result(result_nav, "/lifecycle_manager_navigation/manage_nodes");

  //STEP 4
  nav_node->send_navigate_goal(goal_point);

  rclcpp::sleep_for(3s);
  RCLCPP_INFO(nav_node->get_logger(),"Init manual Nav");

  while (rclcpp::ok() && nav_node->getIsInCorridor())
  {
    // Processa i callback: lidar, ecc.
    rclcpp::spin_some(nav_node);

    nav_node->manual_nav();
    rclcpp::sleep_for(150ms);

    RCLCPP_INFO(
      nav_node->get_logger(),
      "IsInCorridor = %s",
      nav_node->getIsInCorridor() ? "true" : "false"
    );
  }

  nav_node->manual_stop();
  rclcpp::sleep_for(5s);

  //STEP 5
  const int circles_reqs = 4;

  for(int i = 1; i <= circles_reqs; i++)
  {
    RCLCPP_INFO(nav_node->get_logger(),"Calling /detect_circles number: %d time", i);
    nav_node->call_detect_circles();
    rclcpp::sleep_for(1s);
  }

  rclcpp::shutdown();
  return 0;
}