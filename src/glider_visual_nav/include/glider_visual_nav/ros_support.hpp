#pragma once
#include "glider_visual_nav/visual_odometry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>
#include <stdexcept>
#include <utility>

namespace glider_visual_nav
{
template<class T>
T parameter(rclcpp::Node & node, const std::string & name, const T & value)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;  // changing calibration/mode requires a coordinated estimator restart
  return node.declare_parameter<T>(name, value, descriptor);
}
inline double seconds(const builtin_interfaces::msg::Time & time)
{return static_cast<double>(time.sec) + static_cast<double>(time.nanosec) * 1e-9;}
inline builtin_interfaces::msg::Time rosStamp(double value)
{return rclcpp::Time(static_cast<int64_t>(std::llround(value * 1e9)), RCL_ROS_TIME);}
inline bool validVector(const cv::Vec3d & value)
{return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);}
inline bool validCovariance(const cv::Matx33d & covariance)
{
  for (double value : covariance.val) {if (!std::isfinite(value)) {return false;}}
  if (cv::norm(covariance - covariance.t()) > 1e-7) {return false;}
  cv::Mat eigenvalues;
  cv::eigen(cv::Mat(covariance), eigenvalues);
  return eigenvalues.at<double>(2) >= -1e-10;
}
struct ImuSample
{
  tf2::Quaternion orientation;
  cv::Vec3d gyro;
};
inline std::optional<ImuSample> imuSample(const sensor_msgs::msg::Imu & msg, const std::string & frame)
{
  if (msg.header.frame_id != frame || msg.orientation_covariance[0] < 0 ||
    msg.angular_velocity_covariance[0] < 0) {return std::nullopt;}
  const auto & q = msg.orientation;
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  const cv::Vec3d gyro(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
  if (!std::isfinite(quat.length2()) || std::abs(quat.length2() - 1.0) > 0.1 || !validVector(gyro))
  {return std::nullopt;}
  quat.normalize();
  return ImuSample{quat, gyro};
}
inline ImuSample interpolateImu(const ImuSample & a, const ImuSample & b, double t)
{return {a.orientation.slerp(b.orientation, t), a.gyro * (1 - t) + b.gyro * t};}
inline MotionSample motionSample(const ImuSample & sample)
{
  MotionSample out;
  double unused_yaw;
  tf2::Matrix3x3(sample.orientation).getRPY(out.roll, out.pitch, unused_yaw);
  out.angular_velocity_body = sample.gyro;
  return out;
}
template<class T>
class TimeBuffer
{
public:
  void push(double stamp, const T & value)
  {
    if (!std::isfinite(stamp) || stamp < 0) {return;}
    if (!data_.empty() && stamp < data_.back().first) {data_.clear();}
    if (!data_.empty() && stamp == data_.back().first) {data_.back().second = value; return;}
    data_.emplace_back(stamp, value);
    while (data_.size() > 600) {data_.pop_front();}
  }
  template<class Interpolator>
  std::optional<T> sample(double stamp, double max_age, Interpolator interpolate) const
  {
    if (data_.empty()) {return std::nullopt;}
    const auto next = std::lower_bound(data_.begin(), data_.end(), stamp,
      [](const auto & item, double time) {return item.first < time;});
    if (next == data_.begin()) {
      return std::abs(next->first - stamp) <= max_age ? std::optional<T>(next->second) : std::nullopt;
    }
    if (next == data_.end()) {
      return stamp - data_.back().first <= max_age ? std::optional<T>(data_.back().second) : std::nullopt;
    }
    const auto before = std::prev(next);
    if (stamp - before->first > max_age || next->first - stamp > max_age) {return std::nullopt;}
    return interpolate(before->second, next->second, (stamp - before->first) / (next->first - before->first));
  }
  // Integrate the piecewise-linear gyro over a fully bracketed exposure interval.
  template<class Projection>
  std::optional<cv::Vec3d> meanVector(double start, double end, double max_gap, Projection project) const
  {
    if (!(end > start) || data_.empty() || start < data_.front().first ||
      end > data_.back().first) {return std::nullopt;}
    auto upper = std::upper_bound(data_.begin(), data_.end(), start,
      [](double t, const auto & item) {return t < item.first;});
    cv::Vec3d integral(0, 0, 0);
    double cursor = start;
    while (cursor < end) {
      const auto lower = std::prev(upper);
      const double gap = upper->first - lower->first;
      if (gap > max_gap) {return std::nullopt;}
      const double stop = std::min(end, upper->first);
      const auto a = project(lower->second), b = project(upper->second);
      const auto mid = a + (b - a) * ((0.5 * (cursor + stop) - lower->first) / gap);
      integral += mid * (stop - cursor);
      cursor = stop;
      ++upper;
    }
    return integral / (end - start);
  }
private:
  std::deque<std::pair<double, T>> data_;
};
inline cv::Vec3d offsetParameter(rclcpp::Node & node)
{
  const auto values = parameter(node, "camera_offset_body", std::vector<double>{0.0, 0.0, 0.0});
  if (values.size() != 3) {throw std::invalid_argument("camera_offset_body needs three metres");}
  const cv::Vec3d result(values[0], values[1], values[2]);
  if (!validVector(result)) {throw std::invalid_argument("invalid camera offset");}
  return result;
}
}  // namespace glider_visual_nav
