#include "glider_visual_nav/ros_support.hpp"
#include "glider_visual_nav/nodes.hpp"
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>

namespace glider_visual_nav
{
class NavigationNode : public rclcpp::Node
{
public:
  explicit NavigationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("navigation_node", options)
  {
    mode_ = orientationMode(parameter(*this, "orientation_mode", std::string("homography_mag")));
    body_frame_ = parameter(*this, "body_frame", std::string("base_link"));
    world_frame_ = parameter(*this, "world_frame", std::string("odom"));
    declination_ = parameter(*this, "declination", 0.0);
    max_age_ = parameter(*this, "max_sensor_age", 0.15);
    dynamic_height_=parameter(*this,"dynamic_height",false);
    if (dynamic_height_) {
      height_sub_=create_subscription<geometry_msgs::msg::PointStamped>("glider/barometric_height",rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
          if (msg->header.frame_id=="odom" && std::isfinite(msg->point.z)) {heights_.push(seconds(msg->header.stamp),msg->point.z);}
        });
    }
    yaw_variance_ = parameter(*this, "yaw_variance", 0.0025);
    tilt_variance_ = parameter(*this, "tilt_variance", 0.0025);
    min_field_ = parameter(*this, "min_field_tesla", 1e-6);
    max_field_ = parameter(*this, "max_field_tesla", 1e-3);
    const double max_dt = parameter(*this, "max_dt", 0.25);
    camera_offset_ = offsetParameter(*this);
    reckoner_ = std::make_unique<DeadReckoner>(parameter(*this, "h", 40.0), max_dt, yaw_variance_);
    if (body_frame_.empty() || world_frame_.empty() || body_frame_ == world_frame_ ||
      !std::isfinite(max_age_) || max_age_ <= 0 || !std::isfinite(declination_) ||
      !std::isfinite(tilt_variance_) || tilt_variance_ < 0 || !std::isfinite(min_field_) ||
      !std::isfinite(max_field_) || min_field_ <= 0 || max_field_ <= min_field_)
    {throw std::invalid_argument("invalid navigation parameters");}
    publisher_ = create_publisher<nav_msgs::msg::Odometry>("glider/odometry", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    magnetic_sub_ = create_subscription<sensor_msgs::msg::MagneticField>("glider/magnetic_field", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::MagneticField::ConstSharedPtr msg) {
        const cv::Vec3d field(msg->magnetic_field.x, msg->magnetic_field.y, msg->magnetic_field.z);
        if (msg->header.frame_id != body_frame_ || !validVector(field) ||
          cv::norm(field) < min_field_ || cv::norm(field) > max_field_)
        {warn("magnetometer must be valid base_link FLU in tesla"); return;}
        magnetic_.push(seconds(msg->header.stamp), field);
      });
    if (mode_ == OrientationMode::ImuAided) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("glider/imu", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          const auto value = imuSample(*msg, body_frame_);
          if (value) {imu_.push(seconds(msg->header.stamp), *value);}
        });
    }
    velocity_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "glider/visual_velocity", rclcpp::QoS(10),
      [this](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg) {velocity(*msg);});
  }
private:
  void velocity(const geometry_msgs::msg::TwistWithCovarianceStamped & msg)
  {
    const double stamp = seconds(msg.header.stamp);
    if (msg.header.frame_id != body_frame_ || stamp < 0 ||
      (now().seconds() > 0 && std::abs(now().seconds() - stamp) > max_age_))
    {warn("invalid visual velocity frame or stale timestamp"); return;}
    const auto field = magnetic_.sample(stamp, max_age_,
      [](const cv::Vec3d & a, const cv::Vec3d & b, double t) {return a * (1 - t) + b * t;});
    if (!field) {warn("no time-aligned magnetic field; odometry withheld"); return;}
    MotionSample motion;
    if (mode_ == OrientationMode::ImuAided) {
      const auto sample = imu_.sample(stamp, max_age_, interpolateImu);
      if (!sample) {warn("no time-aligned IMU; odometry withheld"); return;}
      motion = motionSample(*sample);
    }
    cv::Matx33d covariance;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {covariance(r, c) = msg.twist.covariance[r * 6 + c];}
    }
    if (!validCovariance(covariance)) {warn("invalid velocity covariance"); return;}
    const auto & v = msg.twist.twist.linear;
    double yaw;
    try {yaw = magneticYaw(*field, motion.roll, motion.pitch, declination_);}
    catch (const std::exception & e) {warn(e.what()); return;}
    if (last_stamp_ && stamp < *last_stamp_) {
      reckoner_->reset(); last_stamp_.reset();
      warn("time rewind: local odometry origin reset");
    }
    if (!reckoner_->update(stamp, cv::Vec3d(v.x, v.y, v.z), covariance, yaw, motion.roll, motion.pitch)) {return;}
    const auto & state = reckoner_->state();
    nav_msgs::msg::Odometry out;
    out.header = msg.header; out.header.frame_id = world_frame_; out.child_frame_id = body_frame_;
    out.pose.pose.position.x = state.position[0]; out.pose.pose.position.y = state.position[1];
    const auto rot = rotationZ(yaw) * rotationY(motion.pitch) * rotationX(motion.roll);
    out.pose.pose.position.z = state.position[2] - (rot * camera_offset_)[2];
    if (dynamic_height_) {
      const auto height=heights_.sample(stamp,max_age_,[](double a,double b,double t) {return a*(1-t)+b*t;});
      if (!height) {warn("missing measured barometric height"); return;}
      out.pose.pose.position.z=*height;
    }
    tf2::Quaternion q; q.setRPY(motion.roll, motion.pitch, yaw);
    out.pose.pose.orientation.x = q.x(); out.pose.pose.orientation.y = q.y();
    out.pose.pose.orientation.z = q.z(); out.pose.pose.orientation.w = q.w();
    out.twist = msg.twist;
    // Yaw is in pose; angular.z is an angular velocity, never the heading angle.
    if (mode_ == OrientationMode::ImuAided) {
      out.twist.twist.angular.x = motion.angular_velocity_body[0];
      out.twist.twist.angular.y = motion.angular_velocity_body[1];
      out.twist.twist.angular.z = motion.angular_velocity_body[2];
    } else if (last_stamp_ && !state.discontinuity && stamp > *last_stamp_) {
      out.twist.twist.angular.z = wrapAngle(yaw - last_yaw_) / (stamp - *last_stamp_);
    }
    for (int r = 0; r < 2; ++r) {
      for (int c = 0; c < 2; ++c) {out.pose.covariance[r * 6 + c] = state.position_covariance(r, c);}
    }
    out.pose.covariance[14] = dynamic_height_ ? .25 : 1e6;
    out.pose.covariance[21] = mode_ == OrientationMode::ImuAided ? tilt_variance_ : 1e6;
    out.pose.covariance[28] = mode_ == OrientationMode::ImuAided ? tilt_variance_ : 1e6;
    out.pose.covariance[35] = yaw_variance_;
    publisher_->publish(out);
    geometry_msgs::msg::TransformStamped transform;
    transform.header = out.header; transform.child_frame_id = body_frame_;
    transform.transform.translation.x = out.pose.pose.position.x;
    transform.transform.translation.y = out.pose.pose.position.y;
    transform.transform.translation.z = out.pose.pose.position.z;
    transform.transform.rotation = out.pose.pose.orientation;
    tf_->sendTransform(transform);
    last_stamp_ = stamp; last_yaw_ = yaw;
    if (state.discontinuity) {warn("velocity gap: position held across missing interval, covariance inflated");}
  }
  void warn(const std::string & message)
  {RCLCPP_WARN_THROTTLE(get_logger(), diagnostic_clock_, 3000, "%s", message.c_str());}
  rclcpp::Clock diagnostic_clock_{RCL_STEADY_TIME};
  OrientationMode mode_;
  bool dynamic_height_{false}; TimeBuffer<double> heights_;
  std::string body_frame_, world_frame_;
  double declination_, max_age_, yaw_variance_, tilt_variance_, min_field_, max_field_, last_yaw_{0};
  cv::Vec3d camera_offset_;
  std::optional<double> last_stamp_;
  TimeBuffer<cv::Vec3d> magnetic_;
  TimeBuffer<ImuSample> imu_;
  std::unique_ptr<DeadReckoner> reckoner_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr height_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_sub_;
};
rclcpp::Node::SharedPtr makeNavigationNode(const rclcpp::NodeOptions & options)
{return std::make_shared<NavigationNode>(options);}
}  // namespace glider_visual_nav
#ifndef GLIDER_VISUAL_NAV_NODE_LIBRARY
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); int status = 0;
  try {rclcpp::spin(std::make_shared<glider_visual_nav::NavigationNode>());}
  catch (const std::exception & e) {RCLCPP_FATAL(rclcpp::get_logger("navigation_node"), "%s", e.what()); status = 1;}
  rclcpp::shutdown(); return status;
}
#endif
