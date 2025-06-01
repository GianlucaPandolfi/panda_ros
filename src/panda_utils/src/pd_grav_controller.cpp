#include "panda_interfaces/msg/joints_effort.hpp"
#include "panda_interfaces/msg/joints_pos.hpp"
#include "panda_utils/constants.hpp"
#include "panda_utils/robot_model.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_eigen/tf2_eigen/tf2_eigen.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <chrono>
#include <memory>
#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_lifecycle/rclcpp_lifecycle/lifecycle_node.hpp>
#include <thread>

template <typename messageT> using Publisher = rclcpp::Publisher<messageT>;
using panda_interfaces::msg::JointsEffort;
using panda_interfaces::msg::JointsPos;
using sensor_msgs::msg::JointState;
auto DEFAULT_URDF_PATH =
    ament_index_cpp::get_package_share_directory("panda_world") +
    panda_constants::panda_model_effort;

class PDGravController : public rclcpp_lifecycle::LifecycleNode {

public:
  PDGravController(const std::string urdf_robot_path = DEFAULT_URDF_PATH)
      : rclcpp_lifecycle::LifecycleNode(
            panda_interface_names::pd_grav_controller_node_name),
        panda(urdf_robot_path, true) {

    // Declare parameters
    this->declare_parameter<double>("Kp", 3750.0);
    this->declare_parameter<double>("Kd", 750.0);
    this->declare_parameter<double>("control_freq", 1000.0);
    this->declare_parameter<bool>("clamp", true);

    // Get parameters
    Kp = this->get_parameter("Kp").as_double();
    Kd = this->get_parameter("Kd").as_double();
    control_loop_rate = std::make_shared<rclcpp::Rate>(
        this->get_parameter("control_freq").as_double(), this->get_clock());
    clamp = this->get_parameter("clamp").as_bool();

    // Taking joint limits

    RCLCPP_INFO(this->get_logger(), "Getting effort limits");
    effort_limits = panda.getModel().effortLimit;

    RCLCPP_INFO(this->get_logger(), "Getting joint pos limits");
    joint_min_limits = panda.getModel().lowerPositionLimit;
    joint_max_limits = panda.getModel().upperPositionLimit;

    auto set_joint_state = [this](const JointState msg) {
      current_joint_config = msg;
    };

    robot_joint_states_sub = this->create_subscription<JointState>(
        panda_interface_names::joint_state_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS, set_joint_state);

    auto set_desired_joint_config = [this](const JointsPos msg) {
      RCLCPP_DEBUG(this->get_logger(), "Setting desired config");
      desired_joints_config.resize(msg.joint_values.size());
      for (size_t i = 0; i < msg.joint_values.size(); i++) {

        desired_joints_config[i] = msg.joint_values[i];
      }
      this->clamp_joint_config();
    };

    desired_joints_config_sub = this->create_subscription<JointsPos>(
        panda_interface_names::panda_pos_cmd_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS, set_desired_joint_config);

    robot_joint_efforts_pub = this->create_publisher<JointsEffort>(
        panda_interface_names::panda_effort_cmd_topic_name,
        panda_interface_names::DEFAULT_TOPIC_QOS);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO_STREAM(
        get_logger(),
        "Configuring node "
            << this->get_name() << " using "
            << (this->get_clock()->ros_time_is_active() ? "simulation clock"
                                                        : "system clock")
            << " and rate clock "
            << (this->control_loop_rate->get_type() == RCL_ROS_TIME
                    ? "simulation clock"
                    : "system clock")
            << ", and control loop rate (Hz) "
            << 1.0 / (control_loop_rate->period().count() * 1e-9)
            << " with Kp = " << Kp << " and Kd = " << Kd);

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {

    RCLCPP_INFO(get_logger(), "Activating...");
    // Set desired pos to last joint states if desired is nullptr
    if (desired_joints_config.size() == 0) {

      RCLCPP_INFO(this->get_logger(),
                  "Setting desired config to latest robot config");
      desired_joints_config.resize(current_joint_config.name.size());

      for (size_t i = 0; i < current_joint_config.name.size(); i++) {
        desired_joints_config[i] = current_joint_config.position[i];
      }
    }

    // Start control loop thread and keep thread id to shutdown it
    start_flag.store(true);
    control_thread = std::thread{std::bind(&PDGravController::control, this)};
    RCLCPP_INFO(this->get_logger(), "Started control thread");

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Deactivating...");
    // Stop thread and join it
    start_flag.store(false);
    control_thread.join();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Cleaning up...");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
    RCLCPP_INFO(get_logger(), "Shutting down...");
    // Stop thread and join it
    start_flag.store(false);
    control_thread.join();
    return CallbackReturn::SUCCESS;
  }

private:
  // Subscribers
  rclcpp::Subscription<JointState>::SharedPtr robot_joint_states_sub{};
  rclcpp::Subscription<JointsPos>::SharedPtr desired_joints_config_sub{};

  // Commands publisher
  Publisher<JointsEffort>::SharedPtr robot_joint_efforts_pub{};

  // Robot related variables
  panda::RobotModel panda;
  JointState current_joint_config{};
  Eigen::VectorXd desired_joints_config{};
  Eigen::VectorXd effort_limits{};
  Eigen::VectorXd joint_min_limits{};
  Eigen::VectorXd joint_max_limits{};

  // Control loop related variables
  rclcpp::Rate::SharedPtr control_loop_rate;
  double Kp{};
  double Kd{};
  std::thread control_thread;
  std::atomic<bool> start_flag{false};
  bool clamp;

  void publish_efforts(const Eigen::VectorXd &efforts) {
    JointsEffort efforts_cmd;
    for (int i = 0; i < efforts.size(); i++) {
      efforts_cmd.effort_values[i] = efforts[i];
    }
    robot_joint_efforts_pub->publish(efforts_cmd);
  }
  void control();
  void clamp_control(Eigen::VectorXd &control_input) {

    for (int i = 0; i < control_input.size(); i++) {
      if (control_input[i] > effort_limits[i]) {
        control_input[i] = effort_limits[i];
      } else if (control_input[i] < -effort_limits[i]) {
        control_input[i] = -effort_limits[i];
      }
    }
  }

  void clamp_joint_config() {
    for (int i = 0; i < this->desired_joints_config.size(); ++i) {
      this->desired_joints_config[i] = std::min(
          std::max(this->desired_joints_config[i], this->joint_min_limits[i]),
          this->joint_max_limits[i]);
    }
  }
};

void PDGravController::control() {
  // PD + gravity compensation controller
  //
  //

  Eigen::VectorXd gravity_vec;
  Eigen::VectorXd control_input;

  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Current desired pose at initial loop: " << desired_joints_config);

  while (start_flag.load() && rclcpp::ok()) {
    // Update robot model
    //
    Eigen::VectorXd current_joints_config_vec =
        Eigen::VectorXd::Map(current_joint_config.position.data(),
                             current_joint_config.position.size());
    Eigen::VectorXd current_joints_speed =
        Eigen::VectorXd::Map(current_joint_config.velocity.data(),
                             current_joint_config.velocity.size());

    panda.computeAll(current_joints_config_vec, current_joints_speed);
    // Calculate quantities for control
    //
    gravity_vec = panda.getGravityVector(current_joints_config_vec);
    control_input = gravity_vec +
                    Kp * (desired_joints_config - current_joints_config_vec)
                    - Kd * current_joints_speed;
    // control_input = -gravity_vec;

    // Clamping control input
    if (clamp) {
      clamp_control(control_input);
    }

    // Apply control
    //
    RCLCPP_DEBUG_STREAM(this->get_logger(), control_input);
    publish_efforts(control_input);
    // Sleep
    control_loop_rate->sleep();
  }

  if (!rclcpp::ok()) {
    RCLCPP_INFO(this->get_logger(), "Requested shutdown");
    return;
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PDGravController>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
