#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "panda_interfaces/srv/calculate_jacobian.hpp"
#include "panda_interfaces/srv/clik.hpp"
#include "panda_interfaces/srv/forward_kine.hpp"
#include "panda_utils/constants.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <chrono>
#include <cstddef>
#include <memory>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/utilities.hpp>

using namespace std::chrono_literals;
using Pose = geometry_msgs::msg::Pose;
using Vector7 = Eigen::Vector<double, 7>;
using Vector6 = Eigen::Vector<double, 6>;
template <int ROWS, int COLS> using Matrix = Eigen::Matrix<double, ROWS, COLS>;
using namespace panda_interfaces::srv;

class CLIKServer : public rclcpp::Node {
public:
  CLIKServer() : Node("clik_server") {
    // Initialize clients
    const int clients_qos = 10;
    const int service_qos = 10;

    fkine_client = this->create_client<ForwardKine>(
        panda_interface_names::forward_kine_service_name, clients_qos,
        callback_group);

    jacobian_client = this->create_client<CalculateJacobian>(
        panda_interface_names::jacob_calc_service_name, clients_qos,
        callback_group);

    while (!jacobian_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(), "Jacobian service not ready yet");
    }

    while (!fkine_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(),
                  "Forward kinematics client not ready yet");
    }

    auto clik_calc = [this](const CLIK_Request::SharedPtr request,
                            CLIK_Response::SharedPtr response) {
      // Get parameters
      double t_camp = request->params.t_camp;
      double gamma = request->params.gamma / t_camp;
      double error_tolerance = request->params.error_tolerance;
      int iters = request->params.iters;
      RCLCPP_DEBUG(this->get_logger(), "Got parameters");

      // Get initial and final pose
      Vector7 current_joint_config = {
          request->joints.joint_values[0], request->joints.joint_values[1],
          request->joints.joint_values[2], request->joints.joint_values[3],
          request->joints.joint_values[4], request->joints.joint_values[5],
          request->joints.joint_values[6]};
      auto transforms = this->get_transforms(current_joint_config);
      Pose current_pose = this->get_pose(transforms);

      Pose final_pose = request->final_pose;
      Eigen::Vector3d final_translation_vec;
      final_translation_vec.x() = final_pose.position.x;
      final_translation_vec.y() = final_pose.position.y;
      final_translation_vec.z() = final_pose.position.z;

      Eigen::Quaterniond final_quaternion{
          final_pose.orientation.w, final_pose.orientation.x,
          final_pose.orientation.y, final_pose.orientation.z};
      final_quaternion.normalize();
      RCLCPP_DEBUG(this->get_logger(), "Initialized pose");

      // Initialize loop variables
      Vector6 error(6);
      double error_norm{1e6};
      long loop_iter{0};
      double lambda = 1e-2;
      RCLCPP_DEBUG(this->get_logger(),
                   "Initialized loop variables, starting loop");

      // Calculate joint configuration (CLIK)
      while ((error_norm > error_tolerance) && loop_iter < iters) {

        ////////////////////////////////////////////////////////////////////////////////////
        /// REMEMBER TO ALWAYS CALL NORMALIZE ON QUATERNION, IF NOT CALLED ALL
        /// THE OPERATIONS ARE UNDEFINED BEHAVIOUR
        ////////////////////////////////////////////////////////////////////////////////////

        transforms = this->get_transforms(current_joint_config);
        auto current_jacob = this->get_jacobian(transforms);
        current_pose = this->get_pose(transforms);
        RCLCPP_DEBUG(this->get_logger(), "Got updated poses");

        // Updating errors
        Eigen::Quaterniond current_quaternion{
            current_pose.orientation.w, current_pose.orientation.x,
            current_pose.orientation.y, current_pose.orientation.z};
        current_quaternion.normalize();
        auto error_quat =
            (final_quaternion * current_quaternion.inverse()).normalized();

        Eigen::Vector3d current_translation_vec{};
        current_translation_vec.x() = current_pose.position.x;
        current_translation_vec.y() = current_pose.position.y;
        current_translation_vec.z() = current_pose.position.z;
        auto error_translation =
            final_translation_vec - current_translation_vec;

        error[0] = error_translation[0];
        error[1] = error_translation[1];
        error[2] = error_translation[2];
        error[3] = error_quat.x();
        error[4] = error_quat.y();
        error[5] = error_quat.z();

        std::cout << "Errors: ";
        for (int i = 0; i < error.size(); i++) {
          std::cout << error[i] << ", ";
        }
        std::cout << std::endl;

        error_norm = error.norm();
        RCLCPP_INFO(this->get_logger(), "Error norm: %f", error_norm);
        RCLCPP_DEBUG(this->get_logger(), "Updated errors");

        // Apply algorithm
        current_joint_config =
            current_joint_config +
            t_camp * ((current_jacob.transpose() *
                       (current_jacob * current_jacob.transpose() +
                        lambda * Eigen::Matrix<double, 6, 6>::Identity())
                           .inverse()) *
                      (gamma * error));
        RCLCPP_DEBUG(this->get_logger(), "Calculated next joints config");

        loop_iter++;
      };

      RCLCPP_DEBUG(this->get_logger(), "Constructing response");
      // Return response
      for (size_t i = 0; i < response->joints.joint_values.size(); i++) {
        response->joints.joint_values[i] = current_joint_config[i];
      }
    };

    clik_service = this->create_service<CLIK>(
        panda_interface_names::clik_service_name, clik_calc, service_qos);
    RCLCPP_INFO(this->get_logger(), "Service CLIK ready");
  }

private:
  std::shared_ptr<rclcpp::Service<CLIK>> clik_service;
  std::shared_ptr<rclcpp::Client<ForwardKine>> fkine_client;
  std::shared_ptr<rclcpp::Client<CalculateJacobian>> jacobian_client;
  rclcpp::CallbackGroup::SharedPtr callback_group;
  const tf2_msgs::msg::TFMessage get_transforms(const Vector7 joints);
  const geometry_msgs::msg::Pose
  get_pose(const tf2_msgs::msg::TFMessage &transforms);
  const Eigen::Matrix<double, 6, 7>
  get_jacobian(const tf2_msgs::msg::TFMessage &transforms);
};

const tf2_msgs::msg::TFMessage
CLIKServer::get_transforms(const Vector7 joints) {

  RCLCPP_DEBUG(this->get_logger(), "Entered get transforms");
  ForwardKine_Request::SharedPtr request =
      std::make_shared<ForwardKine_Request>();

  auto trans_node = rclcpp::Node::make_shared("transforms_call_node");
  auto client = trans_node->create_client<ForwardKine>(
      panda_interface_names::forward_kine_service_name, 10);

  RCLCPP_DEBUG(this->get_logger(), "Assigning joint values to request");
  for (size_t i = 0; i < request->joints.joint_values.size(); i++) {
    request->joints.joint_values[i] = joints[i];
  }
  RCLCPP_DEBUG(this->get_logger(), "Sending async request");
  auto result = client->async_send_request(request);

  RCLCPP_DEBUG(this->get_logger(), "Spinning node for call");
  if (rclcpp::spin_until_future_complete(trans_node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "Service called, returning transforms");
    return result.get()->tfs;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Service aborted call");
    return tf2_msgs::msg::TFMessage();
  };
}

const geometry_msgs::msg::Pose
CLIKServer::get_pose(const tf2_msgs::msg::TFMessage &transforms) {
  Eigen::Isometry3d trans_to_end = Eigen::Isometry3d::Identity();
  for (auto trans : transforms.transforms) {
    trans_to_end = trans_to_end * tf2::transformToEigen(trans);
  }
  Pose pose;
  auto rotation = trans_to_end.rotation();
  Eigen::Quaterniond quat(rotation);
  quat.normalize();

  pose.position.x = trans_to_end.translation().x();
  pose.position.y = trans_to_end.translation().y();
  pose.position.z = trans_to_end.translation().z();
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  return pose;
}

const Eigen::Matrix<double, 6, 7>
CLIKServer::get_jacobian(const tf2_msgs::msg::TFMessage &transforms) {

  RCLCPP_DEBUG(this->get_logger(), "Entered get jacobian");
  CalculateJacobian_Request::SharedPtr request =
      std::make_shared<CalculateJacobian_Request>();
  request->transformations = transforms;

  RCLCPP_DEBUG(this->get_logger(), "Creating nodes");
  auto jacob_node = rclcpp::Node::make_shared("jacobian_call_node");
  auto client = jacob_node->create_client<CalculateJacobian>(
      panda_interface_names::jacob_calc_service_name, 10);

  auto result = client->async_send_request(request);
  RCLCPP_DEBUG(this->get_logger(), "Sent jacob request");

  if (rclcpp::spin_until_future_complete(jacob_node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "Service called");
    auto jacob_columns = result.get()->jacobian.jacobian_data;
    Matrix<6, 7> jacobian{};
    for (size_t i = 0; i < jacob_columns.size(); i++) {
      Vector6 col{jacob_columns[i].data[0], jacob_columns[i].data[1],
                  jacob_columns[i].data[2], jacob_columns[i].data[3],
                  jacob_columns[i].data[4], jacob_columns[i].data[5]};
      jacobian.col(i) = col;
    }
    return jacobian;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Service aborted call");
    return Matrix<6, 7>{};
  };
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CLIKServer>());
  rclcpp::shutdown();
  return 0;
}
