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

#include "vesc_ackermann/vesc_to_odom.hpp"

#include <cmath>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/qos.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

namespace vesc_ackermann
{

using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using sensor_msgs::msg::Imu;
using vesc_msgs::msg::VescStateStamped;

namespace
{

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace

VescToOdom::VescToOdom(const rclcpp::NodeOptions & options)
: Node("vesc_to_odom_node", options),
  odom_frame_("odom"),
  base_frame_("base_link"),
  publish_tf_(true),
  use_simtime_(false),
  x_(0.0),
  y_(0.0),
  yaw_(0.0),
  has_orientation_(false),
  has_previous_imu_(false)
{
  // get ROS parameters
  odom_frame_ = declare_parameter("odom_frame", odom_frame_);
  base_frame_ = declare_parameter("base_frame", base_frame_);

  speed_to_erpm_gain_ = declare_parameter<double>("speed_to_erpm_gain");
  speed_to_erpm_offset_ = declare_parameter<double>("speed_to_erpm_offset");
  wheelbase_ = declare_parameter<double>("wheelbase");

  publish_tf_ = declare_parameter("publish_tf", publish_tf_);
  // use_simtime_ = declare_parameter("use_sim_time", use_simtime_);

  // parameter_callback_handle_ = add_on_set_parameters_callback(
  //   [this](const std::vector<rclcpp::Parameter> & params) {
  //     rcl_interfaces::msg::SetParametersResult result;
  //     result.successful = true;
  //     for (const auto & param : params) {
  //       if (param.get_name() == "use_sim_time") {
  //         use_simtime_ = param.as_bool();
  //       }
  //     }
  //     return result;
  //   });

  // create odom publisher
  odom_pub_ = create_publisher<Odometry>("odom_wheel", 10);

  // create tf broadcaster
  if (publish_tf_) {
    tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
  }

  // subscribe to vesc state and IMU orientation feedback
  vesc_state_sub_ = create_subscription<VescStateStamped>(
    "sensors/core", 10, std::bind(&VescToOdom::vescStateCallback, this, _1));

  imu_sub_ = create_subscription<Imu>(
    "/imu/data", rclcpp::SensorDataQoS(),
    std::bind(&VescToOdom::imuCallback, this, _1));

  last_orientation_.w = 1.0;
}

void VescToOdom::vescStateCallback(const VescStateStamped::SharedPtr state)
{
  // convert to engineering units
  double current_speed = (state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  if (std::fabs(current_speed) < 0.05) {
    current_speed = 0.0;
  }

  // use current state as last state if this is our first time here
  if (!last_state_) {
    last_state_ = state;
  }

  rclcpp::Time state_time(state->header.stamp);

  if (!has_previous_imu_) {
    while (!imu_buffer_.empty() && rclcpp::Time(imu_buffer_.front().header.stamp) <= state_time) {
      previous_imu_ = imu_buffer_.front();
      imu_buffer_.pop_front();
      has_previous_imu_ = true;
      break;
    }
    if (!has_previous_imu_) {
      last_state_ = state;
      return;
    }
  }

  std::vector<Imu> processed_imus;
  processed_imus.reserve(imu_buffer_.size() + 1);
  if (has_previous_imu_) {
    processed_imus.push_back(previous_imu_);
  }

  while (!imu_buffer_.empty() && rclcpp::Time(imu_buffer_.front().header.stamp) <= state_time) {
    processed_imus.push_back(imu_buffer_.front());
    previous_imu_ = imu_buffer_.front();
    imu_buffer_.pop_front();
    has_previous_imu_ = true;
  }

  if (processed_imus.empty()) {
    last_state_ = state;
    return;
  }

  auto normalize_angle = [](double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  };

  const double half_wheelbase = wheelbase_ * 0.5;
  auto compute_beta = [&](double steering_angle) {
    if (wheelbase_ <= 1e-6) {
      return 0.0;
    }
    return std::atan(std::tan(steering_angle) * (half_wheelbase / wheelbase_));
  };

  double delta_x = 0.0;
  double delta_y = 0.0;
  double body_vx_latest = current_speed;
  double body_vy_latest = 0.0;

  for (size_t i = 1; i < processed_imus.size(); ++i) {
    const auto & prev = processed_imus[i - 1];
    const auto & curr = processed_imus[i];
    double sample_dt = (rclcpp::Time(curr.header.stamp) - rclcpp::Time(prev.header.stamp)).seconds();
    if (sample_dt <= 0.0) {
      continue;
    }
    double yaw_prev = yawFromQuaternion(prev.orientation);
    double yaw_curr = yawFromQuaternion(curr.orientation);
    double yaw_diff = normalize_angle(yaw_curr - yaw_prev);
    double sample_yaw_rate = (sample_dt > 0.0) ? yaw_diff / sample_dt : 0.0;
    double steering_angle = 0.0;
    if (std::fabs(current_speed) > 1e-6) {
      steering_angle = std::atan(wheelbase_ * sample_yaw_rate / current_speed);
    }
    double beta = compute_beta(steering_angle);
    double body_vx = current_speed * std::cos(beta);
    double body_vy = current_speed * std::sin(beta);
    double world_vx =
      body_vx * std::cos(yaw_curr) - body_vy * std::sin(yaw_curr);
    double world_vy =
      body_vx * std::sin(yaw_curr) + body_vy * std::cos(yaw_curr);
    delta_x += world_vx * sample_dt;
    delta_y += world_vy * sample_dt;
    body_vx_latest = body_vx;
    body_vy_latest = body_vy;
  }

  const auto & latest_sample = processed_imus.back();
  double latest_yaw = yawFromQuaternion(latest_sample.orientation);
  double tail_dt = (state_time - rclcpp::Time(latest_sample.header.stamp)).seconds();

  double current_angular_velocity = 0.0;
  if (processed_imus.size() >= 2) {
    double yaw_start = yawFromQuaternion(processed_imus.front().orientation);
    double yaw_end = yawFromQuaternion(latest_sample.orientation);
    double yaw_delta = normalize_angle(yaw_end - yaw_start);
    double yaw_dt = (
      rclcpp::Time(latest_sample.header.stamp) -
      rclcpp::Time(processed_imus.front().header.stamp)).seconds();
    if (yaw_dt > 0.0) {
      current_angular_velocity = yaw_delta / yaw_dt;
    }
  }

  if (tail_dt > 0.0) {
    double yaw_rate_tail = (processed_imus.size() >= 2) ? current_angular_velocity : 0.0;
    double steering_angle_tail = 0.0;
    if (std::fabs(current_speed) > 1e-6) {
      steering_angle_tail = std::atan(wheelbase_ * yaw_rate_tail / current_speed);
    }
    double beta_tail = compute_beta(steering_angle_tail);
    double body_vx = current_speed * std::cos(beta_tail);
    double body_vy = current_speed * std::sin(beta_tail);
    double world_vx =
      body_vx * std::cos(latest_yaw) - body_vy * std::sin(latest_yaw);
    double world_vy =
      body_vx * std::sin(latest_yaw) + body_vy * std::cos(latest_yaw);
    delta_x += world_vx * tail_dt;
    delta_y += world_vy * tail_dt;
    body_vx_latest = body_vx;
    body_vy_latest = body_vy;
  }

  x_ += delta_x;
  y_ += delta_y;
  yaw_ = latest_yaw;
  last_orientation_ = latest_sample.orientation;
  has_orientation_ = true;

  previous_imu_.orientation = last_orientation_;
  previous_imu_.header.stamp = state->header.stamp;
  has_previous_imu_ = true;

  // save state for next time
  last_state_ = state;

  // publish odometry message
  Odometry odom;
  odom.header.frame_id = odom_frame_;
  if (use_simtime_) {
    odom.header.stamp = state->header.stamp;
  } else {
    odom.header.stamp = this->get_clock()->now();
  }
  odom.child_frame_id = base_frame_;

  // Position
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  if (has_orientation_) {
    odom.pose.pose.orientation = last_orientation_;
  } else {
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom.pose.pose.orientation.w = 1.0;
  }

  // Position uncertainty
  /** @todo Think about position uncertainty, perhaps get from parameters? */
  odom.pose.covariance[0] = 0.2;   ///< x
  odom.pose.covariance[7] = 0.2;   ///< y
  odom.pose.covariance[35] = 0.4;  ///< yaw

  // Velocity ("in the coordinate frame given by the child_frame_id")
  odom.twist.twist.linear.x = body_vx_latest;
  odom.twist.twist.linear.y = body_vy_latest;
  odom.twist.twist.angular.z = current_angular_velocity;

  // Velocity uncertainty
  /** @todo Think about velocity uncertainty */

  if (publish_tf_) {
    TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    if (use_simtime_) {
      tf.header.stamp = state->header.stamp;
    } else {
      tf.header.stamp = this->get_clock()->now();
    }
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_pub_->sendTransform(tf);
  }
  odom_pub_->publish(odom);
}

void VescToOdom::imuCallback(const Imu::SharedPtr imu)
{
  imu_buffer_.push_back(*imu);
  // keep buffer from growing unbounded
  while (imu_buffer_.size() > 1000) {
    imu_buffer_.pop_front();
  }
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdom)
