#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"

using namespace std::chrono_literals;
using ServiceT = nav2_msgs::srv::ManageLifecycleNodes;

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
      navigation_client = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_navigation/manage_nodes");
      RCLCPP_INFO(this->get_logger(), "localization client init\n");

      // PUBLISH DELLA POSIZIONE INIZIALE
      // SERVICE CLIENT PER ATTIVARE NAVIGATION

      // ACTION CLIENT PER FAR PARTIRE NAVIGATION

  }

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

private:
  //Class Members
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr localization_client;
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr navigation_client;

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
      "Sending localization lifecycle req. | command: %u",
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

  //STEP 3
  auto result_nav = nav_node->send_navigation_request();
  nav_node->handle_result(result_nav, "/lifecycle_manager_navigation/manage_nodes");

  //STEP 4


  rclcpp::shutdown();
  return 0;
}