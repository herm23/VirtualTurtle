#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include <random>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using ServiceT = nav2_msgs::srv::ManageLifecycleNodes;
using ActionT  = nav2_msgs::action::NavigateToPose;
using GoalHandleActionT = rclcpp_action::ClientGoalHandle<ActionT>;

//ciao sono Angelica

enum class InitSteps {
    START,
    GET_GOAL_FROM_SERVER,      // Ottenere la posizione goal da /goal_pose
    ACTIVATE_LOCALIZATION,     // Richiamare il service di attivazione della localization
    INIT_START_POSITION,       // Inizializzare la posizione iniziale
    ACTIVATE_NAVIGATION,       // Richiamare il service di attivazione della navigation
    SEND_GOAL_ACTION           // Inviare il goal tramite action NavigateToPose
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

  void send_navigate_goal()
  {
    if (!navigate_action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server /navigate_to_pose not available");
      return;
    }

    //NOTE: Definiamo posizione statica, sarà da legere dal server "/goal_pose"
    ActionT::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = this->now();

    goal.pose.pose.position.x = 12.5;
    goal.pose.pose.position.y = -2.6;
    goal.pose.pose.position.z = 0.0;

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
      [this](GoalHandleActionT::SharedPtr,
            const std::shared_ptr<const ActionT::Feedback> feedback)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "NavigateToPose feedback: distance_remaining=%.2f",
          feedback->distance_remaining
        );
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

  

private:
  //Class Members
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr localization_client;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr navigation_client;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  bool amcl_received_ = false;
  geometry_msgs::msg::PoseWithCovarianceStamped last_amcl_pose_;
  rclcpp_action::Client<ActionT>::SharedPtr navigate_action_client_;


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
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<NavigationNode> nav_node = std::make_shared<NavigationNode>();
  
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
  nav_node->send_navigate_goal();

  rclcpp::shutdown();
  return 0;
}