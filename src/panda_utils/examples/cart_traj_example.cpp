#include "ament_index_cpp/ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "panda_interfaces/action/cart_traj.hpp"
#include "panda_interfaces/action/joint_traj.hpp"
#include "panda_interfaces/msg/joints_command.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/robot.hpp"
#include "panda_utils/robot_model.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cstdlib>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <unsupported/Eigen/CXX11/src/Tensor/TensorFFT.h>

using panda_interfaces::action::CartTraj;
using panda_interfaces::action::JointTraj;
using sensor_msgs::msg::JointState;
using namespace std::chrono_literals;
using TrajMove = panda_interfaces::action::JointTraj;
using GoalHandleTrajMove = rclcpp_action::ClientGoalHandle<TrajMove>;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cart_traj_example_node");
  geometry_msgs::msg::Pose delta_pose;
  double total_time = 5.0;

  switch (argc) {

  case 1:
    break;
  case 2: {

    RCLCPP_INFO(node->get_logger(), "Choosing time for cartesian trajectory");
    total_time = atoi(argv[1]);
    break;
  }
  case 5: {

    total_time = atoi(argv[1]);
    delta_pose.position.x = atof(argv[2]);
    delta_pose.position.y = atof(argv[3]);
    delta_pose.position.z = atof(argv[4]);
    RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Choosing time ("
            << total_time
            << ") and desired pose with delta on initial pose (only "
               "translation) for cartesian trajectory; Delta pose position: ["
            << delta_pose.position.x << ", " << delta_pose.position.y << ", "
            << delta_pose.position.z << "]");
    break;
  }

  case 9: {

    total_time = atoi(argv[1]);

    delta_pose.position.x = atof(argv[2]);
    delta_pose.position.y = atof(argv[3]);
    delta_pose.position.z = atof(argv[4]);

    delta_pose.orientation.w = atof(argv[4]);
    delta_pose.orientation.x = atof(argv[2]);
    delta_pose.orientation.y = atof(argv[3]);
    delta_pose.orientation.z = atof(argv[4]);
    RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Choosing time ("
            << total_time
            << ") and desired pose with delta on initial pose (only "
               "translation) for cartesian trajectory; Delta pose position: ["
            << delta_pose.position.x << ", " << delta_pose.position.y << ", "
            << delta_pose.position.z << "]. Delta pose orientaion: ["
            << delta_pose.orientation.w << ", " << delta_pose.orientation.x
            << ", " << delta_pose.orientation.y << ", "
            << delta_pose.orientation.z << "].");
    break;
  }
  default: {

    RCLCPP_ERROR(node->get_logger(),
                 "Wrong number of arguments, shutting down");
    rclcpp::shutdown();
    return 0;
  }
  }

  auto joint_cmd_pub =
      node->create_publisher<panda_interfaces::msg::JointsCommand>(
          panda_interface_names::panda_joint_cmd_topic_name,
          panda_interface_names::DEFAULT_TOPIC_QOS);

  auto cart_traj_action_client = rclcpp_action::create_client<CartTraj>(
      node, panda_interface_names::panda_cart_move_action_name);

  auto DEFAULT_URDF_PATH =
      ament_index_cpp::get_package_share_directory("panda_world") +
      panda_constants::panda_model_effort_no_table;
  panda::RobotModel panda{DEFAULT_URDF_PATH};
  Robot<7> panda_mine(PANDA_DH_PARAMETERS, PANDA_JOINT_TYPES,
                      OrientationConfiguration::UNIT_QUATERNION);
  geometry_msgs::msg::Pose initial_pose;

  RCLCPP_INFO(node->get_logger(), "Waiting for servers...");
  cart_traj_action_client->wait_for_action_server();
  RCLCPP_INFO(node->get_logger(), "Servers UP");

  // Get initial pose
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "Getting initial pose, press ENTER");
    std::cin.ignore();

    sensor_msgs::msg::JointState joints;
    RCLCPP_INFO_STREAM(node->get_logger(), "Acquiring joint states");
    rclcpp::wait_for_message<sensor_msgs::msg::JointState>(
        joints, node, panda_interface_names::joint_state_topic_name, 10s);

    double joints_doub[7];
    for (size_t i = 0; i < 7; i++) {
      joints_doub[i] = joints.position[i];
    }

    RCLCPP_INFO_STREAM(node->get_logger(), "Calculating current pose");
    initial_pose = panda_mine.pose(joints_doub);

    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Initial pose: [" << initial_pose.position.x << ", "
                                         << initial_pose.position.y << ", "
                                         << initial_pose.position.z
                                         << "], Orientation (w, x, y, z): ["
                                         << initial_pose.orientation.w << ", "
                                         << initial_pose.orientation.x << ", "
                                         << initial_pose.orientation.y << ", "
                                         << initial_pose.orientation.z << "]");
  }

  // Calling action server
  {
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Ready to call action server, press ENTER");
    std::cin.ignore();

    panda_interfaces::action::CartTraj_Goal cart_goal;
    cart_goal.initial_pose = initial_pose;
    geometry_msgs::msg::Pose desired_pose;
    desired_pose = initial_pose;
    desired_pose.position.x = desired_pose.position.x + delta_pose.position.x;
    desired_pose.position.y = desired_pose.position.y + delta_pose.position.y;
    desired_pose.position.z = desired_pose.position.z + delta_pose.position.z;

    // desired_pose.orientation.w = SQRT2DIV2;
    // desired_pose.orientation.x = 0.0;
    // desired_pose.orientation.y = SQRT2DIV2;
    // desired_pose.orientation.z = 0.0;
    cart_goal.desired_pose = desired_pose;
    cart_goal.total_time = total_time;

    RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Objective: Desired pose ["
            << desired_pose.position.x << ", " << desired_pose.position.y
            << ", " << desired_pose.position.z
            << "], Orientation (w, x, y, z): [" << desired_pose.orientation.w
            << ", " << desired_pose.orientation.x << ", "
            << desired_pose.orientation.y << ", " << desired_pose.orientation.z
            << "] in " << total_time << "[s]");
    // chiamo l'azione e aspetto che termini
    auto future_goal_handle =
        cart_traj_action_client->async_send_goal(cart_goal);
    if (rclcpp::spin_until_future_complete(node, future_goal_handle) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, GOAL NOT SENT");
      return -1;
    }
    auto future_result =
        cart_traj_action_client->async_get_result(future_goal_handle.get());
    if (rclcpp::spin_until_future_complete(node, future_result) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, NO RESULT");
      return -1;
    }

    // check dello stato dell'azione, se non ho errori lo stato deve essere
    // SUCCEEDED
    if (future_result.get().code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "ERROR, CART TRAJECTORY NOT SUCCEEDED");
      return -1;
    }
  }
}
