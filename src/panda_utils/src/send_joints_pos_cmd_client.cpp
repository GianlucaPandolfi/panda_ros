#include "panda_interfaces/msg/joints_pos.hpp"
#include "panda_interfaces/srv/send_joints_pos_cmd.hpp"
#include "panda_utils/constants.hpp"
#include "std_msgs/msg/float64.hpp"
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <vector>

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  if (argc < 8) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Need all 7 joints values to send");
  }

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("client_joints_cmd_pos");
  auto client = node->create_client<panda_interfaces::srv::SendJointsPosCmd>(
      panda_interface_names::joints_cmd_pos_service_name);
  auto request =
      std::make_shared<panda_interfaces::srv::SendJointsPosCmd_Request>();

  for (std::size_t i = 0; i < request->joint_values.joint_values.size(); i++) {
    RCLCPP_INFO(node->get_logger(), "Argument %zu is %s", i + 1, argv[i + 1]);
    request->joint_values.joint_values[i] = atof(argv[i + 1]);
  }

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_INFO(node->get_logger(), "Interrupted node, exiting");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Service not ready yet");
  };

  RCLCPP_INFO(node->get_logger(), "Sending request");
  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Service called");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service aborted call");
  };

  rclcpp::shutdown();
  return 0;
}
