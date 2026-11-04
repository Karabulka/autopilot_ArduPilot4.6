#include "glider_visual_nav/ros_support.hpp"
#include "glider_visual_nav/nodes.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>

namespace glider_visual_nav
{
class LosControllerNode : public rclcpp::Node
{
public:
  explicit LosControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("los_controller_node", options)
  {
    world_frame_ = parameter(*this, "world_frame", std::string("odom"));
    regime_=parameter(*this,"flight_regime",std::string("cruise_const_alt"));
    if(regime_!="cruise_const_alt" && regime_!="unpowered_glide") {throw std::invalid_argument("invalid flight_regime");}
    glide_speed_=parameter(*this,"glide_speed",16.0);
    speed_gain_=parameter(*this,"glide_speed_pitch_gain",.025);
    if(!std::isfinite(glide_speed_) || glide_speed_<=3 || !std::isfinite(speed_gain_) || speed_gain_<0)
    {throw std::invalid_argument("invalid glide speed controller");}
    measured_height_=parameter(*this,"dynamic_height",false);
    camera_offset_=offsetParameter(*this);
    body_frame_ = parameter(*this, "body_frame", std::string("base_link"));
    desired_height_ = parameter(*this, "target_height", 40.0);
    lookahead_ = parameter(*this, "lookahead", 40.0);
    max_roll_ = parameter(*this, "max_roll", 0.52);
    acceptance_ = parameter(*this, "waypoint_acceptance", 15.0);
    min_speed_ = parameter(*this, "min_guidance_speed", 3.0);
    max_age_ = parameter(*this, "max_sensor_age", 0.15);
    max_dt_ = parameter(*this, "max_dt", 0.25);
    pitch_trim_ = parameter(*this, "pitch_trim_up", 0.02);
    max_pitch_ = parameter(*this, "max_pitch_up", 0.25);
    min_pitch_ = parameter(*this, "min_pitch_up", -0.20);
    max_roll_rate_ = parameter(*this, "max_roll_setpoint_rate", 0.5);
    max_pitch_rate_ = parameter(*this, "max_pitch_setpoint_rate", 0.2);
    loop_ = parameter(*this, "loop_waypoints", true);
    const auto points = parameter(*this, "waypoints", std::vector<double>{0, 0, 200, 0, 200, 200, 0, 200});
    if (points.size() < 4 || points.size() % 2 != 0) {throw std::invalid_argument("waypoints must contain at least two ENU pairs");}
    for (size_t i = 0; i < points.size(); i += 2) {
      if (!std::isfinite(points[i]) || !std::isfinite(points[i + 1])) {throw std::invalid_argument("non-finite waypoint");}
      waypoints_.emplace_back(points[i], points[i + 1]);
    }
    for (size_t i = 1; i < waypoints_.size(); ++i) {
      if (cv::norm(waypoints_[i] - waypoints_[i - 1]) < 1e-6) {throw std::invalid_argument("duplicate adjacent waypoint");}
    }
    if (loop_ && cv::norm(waypoints_.front() - waypoints_.back()) < 1e-6)
    {throw std::invalid_argument("loop closes automatically: do not repeat the first waypoint");}
    for (double value : {desired_height_, lookahead_, max_roll_, acceptance_, min_speed_, max_age_,
      max_dt_, max_pitch_, min_pitch_, pitch_trim_, max_roll_rate_, max_pitch_rate_})
    {if (!std::isfinite(value)) {throw std::invalid_argument("non-finite guidance parameter");}}
    if (desired_height_ <= 0 || lookahead_ <= 0 || max_roll_ <= 0 || max_roll_ >= 1.5 || acceptance_ <= 0 ||
      min_speed_ <= 0 || max_age_ <= 0 || max_dt_ <= 0 || min_pitch_ >= max_pitch_ ||
      pitch_trim_ < min_pitch_ || pitch_trim_ > max_pitch_ || max_roll_rate_ <= 0 || max_pitch_rate_ <= 0)
    {throw std::invalid_argument("invalid guidance bounds");}
    PidConfig pid;
    pid.kp = parameter(*this, "height_kp", 0.02); pid.ki = parameter(*this, "height_ki", 0.001);
    pid.kd = parameter(*this, "height_kd", 0.03);
    pid.integral_limit = parameter(*this, "height_integral_limit", 20.0);
    pid.min_output = min_pitch_ - pitch_trim_; pid.max_output = max_pitch_ - pitch_trim_;
    height_pid_ = std::make_unique<HeightPid>(pid);
    PidConfig glide_pid;
    glide_pid.kp=speed_gain_; glide_pid.ki=parameter(*this,"glide_speed_ki",.01); glide_pid.kd=0;
    glide_pid.min_output=min_pitch_-pitch_trim_; glide_pid.max_output=max_pitch_-pitch_trim_;
    glide_pid.integral_limit=20; glide_pid_=std::make_unique<HeightPid>(glide_pid);
    previous_pitch_up_ = pitch_trim_;
    publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("glider/target_pose", 10);
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>("glider/odometry", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {odometry(*msg);});
  }
private:
  void odometry(const nav_msgs::msg::Odometry & msg)
  {
    const double stamp = seconds(msg.header.stamp);
    if (msg.header.frame_id != world_frame_ || msg.child_frame_id != body_frame_ || stamp < 0 ||
      (now().seconds() > 0 && std::abs(now().seconds() - stamp) > max_age_))
    {warn("invalid or stale odometry; no guidance command"); return;}
    if (last_stamp_ && stamp < *last_stamp_) {
      last_stamp_.reset(); segment_ = 0; completed_ = false; height_pid_->reset();
      previous_roll_ = 0; previous_pitch_up_ = pitch_trim_;
    }
    if (completed_ || (last_stamp_ && stamp <= *last_stamp_)) {return;}
    const auto & p = msg.pose.pose.position;
    const auto & orientation = msg.pose.pose.orientation;
    const auto & velocity = msg.twist.twist.linear;
    tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    if (!std::isfinite(q.length2()) || std::abs(q.length2() - 1.0) > 0.1 ||
      !validVector(cv::Vec3d(p.x, p.y, p.z)) || !validVector(cv::Vec3d(velocity.x, velocity.y, velocity.z)))
    {warn("invalid odometry values"); return;}
    q.normalize();
    const auto v = tf2::Matrix3x3(q) * tf2::Vector3(velocity.x, velocity.y, velocity.z);
    const auto offset_world=tf2::Matrix3x3(q)*tf2::Vector3(camera_offset_[0],camera_offset_[1],camera_offset_[2]);
    const double desired_body_height=desired_height_-offset_world.z();
    const double speed = std::hypot(v.x(), v.y());
    const double dt = last_stamp_ ? stamp - *last_stamp_ : 0.0;
    if (measured_height_ && previous_height_ && dt>0 && dt<=max_dt_) {
      const double alpha=dt/(.35+dt);
      baro_vertical_speed_ += alpha*(std::clamp((p.z-*previous_height_)/dt,-15.,15.)-baro_vertical_speed_);
    }
    previous_height_=p.z;
    if (dt > max_dt_) {height_pid_->reset(); glide_pid_->reset();}
    last_stamp_ = stamp;
    if (speed < min_speed_) {height_pid_->reset(); glide_pid_->reset(); warn("speed below guidance threshold; no command"); return;}
    const cv::Point2d position(p.x, p.y);
    const double course = std::atan2(v.y(), v.x());
    size_t next = (segment_ + 1) % waypoints_.size();
    auto los = lineOfSight(waypoints_[segment_], waypoints_[next], position, course, speed, lookahead_, max_roll_);
    const double segment_length = cv::norm(waypoints_[next] - waypoints_[segment_]);
    if (cv::norm(position - waypoints_[next]) <= acceptance_ ||
      (los.along_track >= segment_length && std::abs(los.cross_track) <= acceptance_))
    {
      if (!loop_ && next == waypoints_.size() - 1) {
        completed_ = true;
        RCLCPP_INFO(get_logger(), "Route complete; target publication stopped (no aircraft stop command)");
        return;
      }
      segment_ = next; next = (segment_ + 1) % waypoints_.size();
      los = lineOfSight(waypoints_[segment_], waypoints_[next], position, course, speed, lookahead_, max_roll_);
    }
    double pitch_up = pitch_trim_;
    if(regime_=="unpowered_glide") {
      if(dt>0 && dt<=max_dt_) {
        pitch_up+=glide_pid_->update(cv::norm(cv::Vec3d(velocity.x,velocity.y,velocity.z))-glide_speed_,0,dt);
      }
    } else if (dt > 0 && dt <= max_dt_) {
      pitch_up += height_pid_->update(desired_body_height - p.z, measured_height_ ? baro_vertical_speed_ : v.z(), dt);
    }
    const double step = dt > 0 && dt <= max_dt_ ? dt : 0.0;
    const double roll = std::clamp(los.desired_roll, previous_roll_ - max_roll_rate_ * step,
      previous_roll_ + max_roll_rate_ * step);
    pitch_up = std::clamp(pitch_up, previous_pitch_up_ - max_pitch_rate_ * step,
      previous_pitch_up_ + max_pitch_rate_ * step);
    previous_roll_ = roll; previous_pitch_up_ = pitch_up;
    geometry_msgs::msg::PoseStamped target;
    target.header = msg.header;
    target.pose.position.x = waypoints_[next].x; target.pose.position.y = waypoints_[next].y;
    target.pose.position.z = regime_=="unpowered_glide" ? p.z : desired_body_height;
    tf2::Quaternion desired; desired.setRPY(roll, -pitch_up, los.desired_course);
    target.pose.orientation.x = desired.x(); target.pose.orientation.y = desired.y();
    target.pose.orientation.z = desired.z(); target.pose.orientation.w = desired.w();
    publisher_->publish(target);
    RCLCPP_INFO_THROTTLE(get_logger(),diagnostic_clock_,3000,"LOS segment=%zu cross_track=%.2f height=%.2f speed=%.2f roll=%.3f pitch_up=%.3f",segment_,los.cross_track,p.z,speed,roll,pitch_up);
  }
  void warn(const std::string & text)
  {RCLCPP_WARN_THROTTLE(get_logger(), diagnostic_clock_, 3000, "%s", text.c_str());}
  rclcpp::Clock diagnostic_clock_{RCL_STEADY_TIME};
  std::string world_frame_, body_frame_;
  std::string regime_; double glide_speed_,speed_gain_;
  bool measured_height_{false}; std::optional<double> previous_height_; double baro_vertical_speed_{0};
  cv::Vec3d camera_offset_;
  double desired_height_, lookahead_, max_roll_, acceptance_, min_speed_, max_age_, max_dt_;
  double pitch_trim_, max_pitch_, min_pitch_, max_roll_rate_, max_pitch_rate_;
  double previous_roll_{0}, previous_pitch_up_{0};
  bool loop_, completed_{false};
  size_t segment_{0};
  std::vector<cv::Point2d> waypoints_;
  std::optional<double> last_stamp_;
  std::unique_ptr<HeightPid> height_pid_;
  std::unique_ptr<HeightPid> glide_pid_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
};
rclcpp::Node::SharedPtr makeLosControllerNode(const rclcpp::NodeOptions & options)
{return std::make_shared<LosControllerNode>(options);}
}  // namespace glider_visual_nav
#ifndef GLIDER_VISUAL_NAV_NODE_LIBRARY
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv); int status = 0;
  try {rclcpp::spin(std::make_shared<glider_visual_nav::LosControllerNode>());}
  catch (const std::exception & e) {RCLCPP_FATAL(rclcpp::get_logger("los_controller_node"), "%s", e.what()); status = 1;}
  rclcpp::shutdown(); return status;
}
#endif
