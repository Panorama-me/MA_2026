#include "sentry/node.hpp"
#include "sentry_2026/io/end_input/msg/send_data.hpp"
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

namespace sentry {
Sentry::Sentry(const rclcpp::NodeOptions &options) : Node("sentry", options) {
  RCLCPP_INFO(get_logger(), "Start sentry!");
  receive_sub_ =
      this->create_subscription<sentry_2026::io::end_input::msg::SendData>(
          "/sentry_to_aim_data", rclcpp::SensorDataQoS(),
          std::bind(&sentry::receive_data, this, std::placeholders::_1));

  send_pub_ =
      this->create_publisher<sentry::msg::Send>("/aim_to_sentry_data", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&Sentry::send_data, this));
  
}

Sentry::receive_data(const sentry::msg::Send::SharedPtr msg) {
  try {
    auto receive_msg =
        std::make_unique<entry_2026::io::end_input::msg::SendData>();
    this->DataNav2Aim.vx = receive_msg->line_vel_x;
    this->DataNav2Aim.vy = receive_msg->line_vel_y;
    this->DataNav2Aim.wz = receive_msg->angle_vel_z;
    this->sum_nav2aim.push_back(this->DataNav2Aim);
  } catch {
    RCLCPP_INFO(get_logger(), "Failed aim receive sentry");
  }
}
Sentry::send_data(){
  auto message =src::vision_2026::sentry::msg::Send();
  /*  data  */
  publisher_->publish(message);
}
} // namespace sentry