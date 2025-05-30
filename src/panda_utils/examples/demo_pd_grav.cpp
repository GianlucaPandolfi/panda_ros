#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "panda_interfaces/msg/joints_effort.hpp"
#include "panda_interfaces/msg/joints_pos.hpp"
#include "panda_utils/constants.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_eigen/tf2_eigen/tf2_eigen.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <array>
#include <chrono>
#include <rclcpp/client.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_lifecycle/rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/rclcpp_lifecycle/transition.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/transition.hpp>
#include <string>
#include <thread>

using panda_interfaces::msg::JointsPos;
using namespace std::chrono_literals;

template <typename FutureT, typename WaitTimeT>
std::future_status wait_for_result(FutureT &future, WaitTimeT time_to_wait) {
  auto end = std::chrono::steady_clock::now() + time_to_wait;
  std::chrono::milliseconds wait_period(100);
  std::future_status status = std::future_status::timeout;
  do {
    auto now = std::chrono::steady_clock::now();
    auto time_left = end - now;
    if (time_left <= std::chrono::seconds(0)) {
      break;
    }
    status =
        future.wait_for((time_left < wait_period) ? time_left : wait_period);
  } while (rclcpp::ok() && status != std::future_status::ready);
  return status;
}

class LifecycleServiceClient : public rclcpp::Node {
public:
  explicit LifecycleServiceClient(const std::string &node_name,
                                  const std::string &lifecycle_node_name)
      : Node(node_name), lifecycle_node_name(lifecycle_node_name) {}

  void init() {
    // Every lifecycle node spawns automatically a couple
    // of services which allow an external interaction with
    // these nodes.
    // The two main important ones are GetState and ChangeState.
    client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
        this->lifecycle_node_name + std::string{"/get_state"});
    client_change_state_ =
        this->create_client<lifecycle_msgs::srv::ChangeState>(
            this->lifecycle_node_name + std::string{"/change_state"});
  }

  /// Requests the current state of the node
  /**
   * In this function, we send a service request
   * asking for the current state of the node
   * lc_talker.
   * If it does return within the given time_out,
   * we return the current state of the node, if
   * not, we return an unknown state.
   * \param time_out Duration in seconds specifying
   * how long we wait for a response before returning
   * unknown state
   */
  unsigned int get_state(std::chrono::seconds time_out = 3s) {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!client_get_state_->wait_for_service(time_out)) {
      RCLCPP_ERROR(get_logger(), "Service %s is not available.",
                   client_get_state_->get_service_name());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // We send the service request for asking the current
    // state of the lc_talker node.
    auto future_result = client_get_state_->async_send_request(request);

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(),
                   "Server time out while getting current state for node %s",
                   lifecycle_node_name.c_str());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    // We have an succesful answer. So let's print the current state.
    if (future_result.get()) {
      RCLCPP_INFO(get_logger(), "Node %s has current state %s.",
                  lifecycle_node_name.c_str(),
                  future_result.get()->current_state.label.c_str());
      return future_result.get()->current_state.id;
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to get current state for node %s",
                   lifecycle_node_name.c_str());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
  }

  /// Invokes a transition
  /**
   * We send a Service request and indicate
   * that we want to invoke transition with
   * the id "transition".
   * By default, these transitions are
   * - configure
   * - activate
   * - cleanup
   * - shutdown
   * \param transition id specifying which
   * transition to invoke
   * \param time_out Duration in seconds specifying
   * how long we wait for a response before returning
   * unknown state
   */
  bool change_state(std::uint8_t transition,
                    std::chrono::seconds time_out = 3s) {
    auto request =
        std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!client_change_state_->wait_for_service(time_out)) {
      RCLCPP_ERROR(get_logger(), "Service %s is not available.",
                   client_change_state_->get_service_name());
      return false;
    }

    // We send the request with the transition we want to invoke.
    auto future_result = client_change_state_->async_send_request(request);

    // Let's wait until we have the answer from the node.
    // If the request times out, we return an unknown state.
    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(),
                   "Server time out while getting current state for node %s",
                   lifecycle_node_name.c_str());
      return false;
    }

    // We have an answer, let's print our success.
    if (future_result.get()->success) {
      RCLCPP_INFO(get_logger(), "Transition %d successfully triggered.",
                  static_cast<int>(transition));
      return true;
    } else {
      RCLCPP_WARN(get_logger(), "Failed to trigger transition %u",
                  static_cast<unsigned int>(transition));
      return false;
    }
  }

private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>>
      client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>
      client_change_state_;
  std::string lifecycle_node_name;
};

/**
 * This is a little independent
 * script which triggers the
 * default lifecycle of a node.
 * It starts with configure, activate,
 * deactivate, activate, deactivate,
 * cleanup and finally shutdown
 */
void callee_script(std::shared_ptr<LifecycleServiceClient> lc_client,
                   rclcpp::Publisher<JointsPos>::SharedPtr des_pos_pub) {
  rclcpp::WallRate time_between_state_changes(0.1); // 10s

  // configure
  {
    if (!lc_client->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // activate
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return;
    }
    if (!lc_client->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
      JointsPos des_config;
      des_config.set__joint_values(std::array<double, 7>{0, 0, 0, 0, 0, 0, 0});
      des_pos_pub->publish(des_config);
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // deactivate
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return;
    }
    if (!lc_client->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // activate it again
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return;
    }
    if (!lc_client->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // and deactivate it again
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return;
    }
    if (!lc_client->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // we cleanup
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return;
    }
    if (!lc_client->change_state(
            lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }

  // and finally shutdown
  // Note: We have to be precise here on which shutdown transition id to call
  // We are currently in the unconfigured state and thus have to call
  // TRANSITION_UNCONFIGURED_SHUTDOWN
  {
    time_between_state_changes.sleep();
    if (!rclcpp::ok()) {
      return;
    }
    if (!lc_client->change_state(lifecycle_msgs::msg::Transition::
                                     TRANSITION_UNCONFIGURED_SHUTDOWN)) {
      return;
    }
    if (!lc_client->get_state()) {
      return;
    }
  }
}

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  auto lc_client = std::make_shared<LifecycleServiceClient>(
      "pd_grav_controller_demo_node",
      panda_interface_names::pd_grav_controller_node_name);
  lc_client->init();

  rclcpp::Publisher<JointsPos>::SharedPtr des_config_pub =
      lc_client->create_publisher<JointsPos>(
          panda_interface_names::panda_pos_cmd_topic_name,
          panda_interface_names::DEFAULT_TOPIC_QOS);
  std::this_thread::sleep_for(2s);

  lc_client->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  RCLCPP_INFO(lc_client->get_logger(), "Transitioned to configure");
  std::this_thread::sleep_for(2s);
  JointsPos des_config;
  des_config.set__joint_values(std::array<double, 7>{0, 0, 0, 0, 0, 0, 0});
  des_config_pub->publish(des_config);

  lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  RCLCPP_INFO(lc_client->get_logger(), "Transitioned to activate");
  std::this_thread::sleep_for(10s);

  lc_client->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  RCLCPP_INFO(lc_client->get_logger(), "Transitioned to deactivate");
  std::this_thread::sleep_for(2s);

  lc_client->change_state(
      lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
  RCLCPP_INFO(lc_client->get_logger(), "Transitioned to shutdown");
  // rclcpp::executors::SingleThreadedExecutor exe;
  // exe.add_node(lc_client);
  //
  // std::shared_future<void> script = std::async(
  //     std::launch::async, std::bind(callee_script, lc_client,
  //     des_config_pub));
  // exe.spin_until_future_complete(script);

  rclcpp::shutdown();

  return 0;
}
