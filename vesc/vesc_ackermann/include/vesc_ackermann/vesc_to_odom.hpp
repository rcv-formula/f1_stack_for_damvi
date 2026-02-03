// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_ACKERMANN__VESC_TO_ODOM_HPP_
#define VESC_ACKERMANN__VESC_TO_ODOM_HPP_

#include <cstddef>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace vesc_ackermann
{

using nav_msgs::msg::Odometry;
using vesc_msgs::msg::VescStateStamped;

class VescToOdom : public rclcpp::Node
{
public:
  explicit VescToOdom(const rclcpp::NodeOptions & options);

private:
  struct ImuIntegrationResult
  {
    double delta_x = 0.0;
    double delta_y = 0.0;
    double latest_yaw = 0.0;
    geometry_msgs::msg::Quaternion latest_orientation;
    double body_vx_latest = 0.0;
    double body_vy_latest = 0.0;
    double angular_velocity = 0.0;
    double accel_speed_estimate = 0.0;
    bool accel_speed_valid = false;
  };

  // ROS parameters
  std::string odom_frame_;
  std::string base_frame_;
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double wheelbase_;
  double imu_x_offset_;
  bool publish_tf_;
  bool use_simtime_;

  // odometry state
  double x_, y_, yaw_;
  geometry_msgs::msg::Quaternion last_orientation_;
  bool has_orientation_;
  bool has_previous_imu_;
  VescStateStamped::SharedPtr last_state_;  ///< Last received state message
  sensor_msgs::msg::Imu previous_imu_;
  std::deque<sensor_msgs::msg::Imu> imu_buffer_;

  // IMU accel calibration/state
  bool imu_calibrated_;
  bool imu_calibration_started_;
  rclcpp::Time imu_calib_start_time_;
  double imu_calib_duration_;
  double imu_ax_bias_;
  double imu_calib_sum_;
  std::size_t imu_calib_count_;
  double accel_stationary_threshold_;
  double yaw_rate_stationary_threshold_;

  // ROS services
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<VescStateStamped>::SharedPtr vesc_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  // ROS callbacks
  void vescStateCallback(const VescStateStamped::SharedPtr state);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu);

  bool collectImuSamplesUpTo(
    const rclcpp::Time & state_time,
    std::vector<sensor_msgs::msg::Imu> & processed_imus);

  ImuIntegrationResult integrateImuSamples(
    const std::vector<sensor_msgs::msg::Imu> & samples,
    const rclcpp::Time & state_time,
    double current_speed) const;

  Odometry createOdomMessage(
    const VescStateStamped::SharedPtr & state,
    const ImuIntegrationResult & imu_result);
};

}  // namespace vesc_ackermann

#endif  // VESC_ACKERMANN__VESC_TO_ODOM_HPP_
