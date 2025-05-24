#include "panda_interfaces/srv/send_joints_pos_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <vector>
#include "panda_utils/constants.hpp"

class SendJointsCmdPosServer : public rclcpp::Node {
public:
  SendJointsCmdPosServer() : Node("send_joints_cmd_pos") {

    // Create publishers to cmd pos topic for position movement
    auto topic_names = {
        "/joint1/cmd_pos", "/joint2/cmd_pos", "/joint3/cmd_pos",
        "/joint4/cmd_pos", "/joint5/cmd_pos", "/joint6/cmd_pos",
        "/joint7/cmd_pos",
    };

    auto topic_qos = 10;
    for (const auto &name : topic_names) {
      auto pub =
          this->create_publisher<std_msgs::msg::Float64>(name, topic_qos);
      joints_pubs.push_back(pub);
    }

    // Create service for publication
    auto send_pos_cmd =
        [this](const std::shared_ptr<
                   panda_interfaces::srv::SendJointsPosCmd_Request>
                   request,
               std::shared_ptr<panda_interfaces::srv::SendJointsPosCmd_Response>
                   response) {
          auto joints_values = request->joint_values.joint_values;
          for (size_t i = 0;
               i < std::min(joints_values.size(), joints_pubs.size()); ++i) {

            double joint_val = joints_values[i];
            auto pub = this->joints_pubs[i];
            std_msgs::msg::Float64 mess;
            mess.data = joint_val;
            pub->publish(mess);
          }

          response->result = response->OK;
        };

    service = this->create_service<panda_interfaces::srv::SendJointsPosCmd>(
        panda_interface_names::joints_cmd_pos_service_name, send_pos_cmd, topic_qos);

    RCLCPP_INFO(this->get_logger(), "Service started");
  }

private:
  std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> joints_pubs;
  rclcpp::Service<panda_interfaces::srv::SendJointsPosCmd>::SharedPtr service;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SendJointsCmdPosServer>());
  rclcpp::shutdown();
  return 0;
}
