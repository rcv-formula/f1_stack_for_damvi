// ackermann_to_vesc.hpp

#ifndef VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_
#define VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using geometry_msgs::msg::Twist;
using std_msgs::msg::Float64;

class AckermannToVesc : public rclcpp::Node
{
public:
  explicit AckermannToVesc(const rclcpp::NodeOptions & options);

private:
  // ROS parameters
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double steering_to_servo_gain_, steering_to_servo_offset_;

  // ROS publishers
  rclcpp::Publisher<Float64>::SharedPtr erpm_pub_;
  rclcpp::Publisher<Float64>::SharedPtr servo_pub_;
  rclcpp::Publisher<AckermannDriveStamped>::SharedPtr ackermann_pub_;

  // ROS subscribers
  rclcpp::Subscription<AckermannDriveStamped>::SharedPtr ackermann_sub_;
  rclcpp::Subscription<Twist>::SharedPtr cmd_vel_sub_;

  // ROS callbacks
  void ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd);
  void cmdVelCallback(const Twist::SharedPtr msg);  // cmd_vel을 처리하는 새로운 콜백 함수 선언
};

}  // namespace vesc_ackermann

#endif  // VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_

