#include "panda_interfaces/msg/joints_pos.hpp"
#include "panda_interfaces/srv/forward_kine.hpp"
#include "panda_utils/constants.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
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

  auto fkine_client = node->create_client<panda_interfaces::srv::ForwardKine>(
      panda_interface_names::forward_kine_service_name);

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

    auto tf = Eigen::Isometry3d::Identity();
    RCLCPP_DEBUG(node->get_logger(), "Defined tf identity");
    auto transforms = result.get()->tfs.transforms;
    RCLCPP_DEBUG(node->get_logger(), "Getting transforms from result");
    for (size_t i = 0; i < transforms.size(); i++) {
      RCLCPP_DEBUG(node->get_logger(), "Multiplicating tf by tf");
      tf = tf * tf2::transformToEigen(transforms[i]);
    }
    RCLCPP_DEBUG(node->get_logger(), "Done mutliplication, printing");
    auto translation = tf.translation();
    auto rotation = tf.rotation();
    std::cout << "Translation: " << std::endl << translation << std::endl;
    std::cout << "Rotation: " << std::endl << rotation << std::endl;
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service aborted call");
  };
}
