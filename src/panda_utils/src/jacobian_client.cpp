#include "panda_interfaces/msg/joints_pos.hpp"
#include "panda_interfaces/srv/calculate_jacobian.hpp"
#include "panda_interfaces/srv/forward_kine.hpp"
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
#include "panda_utils/constants.hpp"

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("jacobian_client_node");

  auto fkine_request =
      std::make_shared<panda_interfaces::srv::ForwardKine_Request>();

  if (argc < 8) {
    RCLCPP_INFO(
        node->get_logger(),
        "Not enough arguments, going with default joints values (all zeros)");
    fkine_request->joints.joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  } else {
    fkine_request->joints.joint_values = {
        atof(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]),
        atof(argv[5]), atof(argv[6]), atof(argv[7])};
  }
  auto jacob_client =
      node->create_client<panda_interfaces::srv::CalculateJacobian>(
panda_interface_names::jacob_calc_service_name);

  auto fkine_client =
      node->create_client<panda_interfaces::srv::ForwardKine>(panda_interface_names::forward_kine_service_name);

  auto jacob_request =
      std::make_shared<panda_interfaces::srv::CalculateJacobian_Request>();

  while (!fkine_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_INFO(node->get_logger(), "Interrupted node, exiting");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Service fkine not ready yet");
  };

  RCLCPP_INFO(node->get_logger(), "Sending request");
  auto result = fkine_client->async_send_request(fkine_request);

  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Service called");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service aborted call");
  };

  jacob_request->transformations = result.get()->tfs;

  while (!jacob_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_INFO(node->get_logger(), "Interrupted node, exiting");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Service jacob not ready yet");
  };

  RCLCPP_INFO(node->get_logger(), "Sending jacob request");
  auto jacob_result = jacob_client->async_send_request(jacob_request);

  if (rclcpp::spin_until_future_complete(node, jacob_result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Service called");
    auto jacob = jacob_result.get()->jacobian.jacobian_data;
    RCLCPP_INFO(node->get_logger(), "Jacobian is: \n[ ");
    for (size_t i = 0; i < 6; i++) {
      for (size_t j = 0; j < jacob.size(); ++j) {
        double num = jacob[j].data[i];
        if (num < 1e-4 && num > 0) {
          num = 0;
        } else if (num < 0 && num > -1e-4) {
          num = 0;
        }
        std::cout << "\t" << num;
      }
      std::cout << std::endl;
    }
    std::cout << " ]";
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service aborted call");
  };

  rclcpp::shutdown();
  return 0;
}
