#include "glider_visual_nav/ros_support.hpp"
#include "glider_visual_nav/nodes.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <opencv2/calib3d.hpp>
#include <memory>

namespace glider_visual_nav
{
class OpticalFlowNode : public rclcpp::Node
{
public:
  explicit OpticalFlowNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("optical_flow_node", options)
  {
    FlowConfig config;
    config.mode = orientationMode(parameter(*this, "orientation_mode", std::string("homography_mag")));
    mode_ = config.mode;
    config.height = parameter(*this, "h", 40.0);
    config.max_features = parameter(*this, "max_features", 300);
    config.min_inliers = parameter(*this, "min_inliers", 12);
    config.fb_threshold = parameter(*this, "fb_threshold", 0.7);
    config.ransac_threshold = parameter(*this, "ransac_threshold", 1.5);
    config.max_lk_error = parameter(*this, "max_lk_error", 30.0);
    config.max_dt = parameter(*this, "max_dt", 0.25);
    config.horizontal_tolerance = parameter(*this, "horizontal_tolerance", 0.05);
    config.velocity_noise_floor = parameter(*this, "velocity_noise_floor", 0.02);
    config.camera_offset_body = offsetParameter(*this);
    odometry_ = std::make_unique<VisualOdometry>(config);
    camera_.fx = parameter(*this, "fx", 539.936); camera_.fy = parameter(*this, "fy", 539.936);
    camera_.cx = parameter(*this, "cx", 640.0); camera_.cy = parameter(*this, "cy", 480.0);
    camera_.validate();
    width_ = parameter(*this, "image_width", 1280); height_ = parameter(*this, "image_height", 960);
    body_frame_ = parameter(*this, "body_frame", std::string("base_link"));
    optical_frame_ = parameter(*this, "optical_frame", std::string("camera_optical_frame"));
    imu_camera_time_offset_ = parameter(*this, "imu_camera_time_offset", 0.0);
    if (!std::isfinite(imu_camera_time_offset_)) {throw std::invalid_argument("invalid IMU time offset");}
    sensor_age_ = parameter(*this, "max_sensor_age", 0.15);
    dynamic_height_ = parameter(*this, "dynamic_height", false);
    camera_offset_ = config.camera_offset_body;
    if (dynamic_height_) {
      height_sub_ = create_subscription<geometry_msgs::msg::PointStamped>("glider/barometric_height", rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
          if (msg->header.frame_id == "odom" && std::isfinite(msg->point.z)) {heights_.push(seconds(msg->header.stamp),msg->point.z);}
        });
    }
    if (width_ <= 0 || height_ <= 0 || body_frame_.empty() || optical_frame_.empty() ||
      body_frame_ == optical_frame_ || !std::isfinite(sensor_age_) || sensor_age_ <= 0)
    {throw std::invalid_argument("invalid frame, image dimensions or sensor age");}
    publisher_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("glider/visual_velocity", 10);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>("camera/camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {cameraInfo(*msg);});
    image_sub_ = create_subscription<sensor_msgs::msg::Image>("camera/image_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {image(msg);});
    if (mode_ == OrientationMode::ImuAided) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("glider/imu", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          const auto value = imuSample(*msg, body_frame_);
          if (value) {imu_.push(seconds(msg->header.stamp), *value);}
        });
    }
    rclcpp::PublisherOptions tf_options;
    tf_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this,
      tf2_ros::StaticBroadcasterQoS(), tf_options);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now(); transform.header.frame_id = body_frame_; transform.child_frame_id = optical_frame_;
    transform.transform.translation.x = config.camera_offset_body[0];
    transform.transform.translation.y = config.camera_offset_body[1];
    transform.transform.translation.z = config.camera_offset_body[2];
    const auto r = opticalToBody();
    tf2::Quaternion q;
    tf2::Matrix3x3(r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0), r(2, 1), r(2, 2)).getRotation(q);
    transform.transform.rotation.x = q.x(); transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z(); transform.transform.rotation.w = q.w();
    static_tf_->sendTransform(transform);
    RCLCPP_INFO(get_logger(), "Visual flow: %dx%d, fx=%.3f fy=%.3f, sim_time=%d; no GPS inputs",
      width_, height_, camera_.fx, camera_.fy, get_parameter("use_sim_time").as_bool());
  }
private:
  void cameraInfo(const sensor_msgs::msg::CameraInfo & msg)
  {
    if (msg.header.frame_id != optical_frame_) {warn("CameraInfo must use optical frame"); return;}
    try {
      CameraModel candidate{msg.k[0], msg.k[4], msg.k[2], msg.k[5], msg.k[1]};
      candidate.validate();
      if (msg.width == 0 || msg.height == 0 || std::abs(msg.k[8] - 1.0) > 1e-9 ||
        std::abs(msg.k[3]) + std::abs(msg.k[6]) + std::abs(msg.k[7]) > 1e-9 ||
        msg.binning_x > 1 || msg.binning_y > 1 || msg.roi.width != 0 || msg.roi.height != 0)
      {throw std::invalid_argument("unsupported CameraInfo dimensions, ROI, binning or K");}
      if (!msg.distortion_model.empty() && msg.distortion_model != "plumb_bob" && msg.distortion_model != "rational_polynomial")
      {throw std::invalid_argument("unsupported distortion model");}
      for (double value : msg.d) {if (!std::isfinite(value)) {throw std::invalid_argument("invalid distortion");}}
      if (!msg.d.empty() && msg.d.size() != 4 && msg.d.size() != 5 && msg.d.size() != 8 && msg.d.size() != 12 && msg.d.size() != 14)
      {throw std::invalid_argument("unsupported distortion coefficient count");}
      if (cv::norm(candidate.matrix() - camera_.matrix()) > 1e-9 || msg.d != distortion_ ||
        static_cast<int>(msg.width) != width_ || static_cast<int>(msg.height) != height_)
      {odometry_->reset(); previous_image_stamp_.reset();}
      camera_ = candidate; distortion_ = msg.d;
      width_ = static_cast<int>(msg.width); height_ = static_cast<int>(msg.height);
      calibrated_ = true;
    } catch (const std::exception & e) {warn(e.what());}
  }
  void image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    const double stamp = seconds(msg->header.stamp);
    RCLCPP_INFO_ONCE(get_logger(), "First image stamp=%.6f clock=%.6f frame=%s",
      stamp, now().seconds(), msg->header.frame_id.c_str());
    if (msg->header.frame_id != optical_frame_ || stamp < 0 ||
      (now().seconds() > 0 && std::abs(now().seconds() - stamp) > sensor_age_))
    {warn("invalid image frame or stale timestamp"); return;}
    if (static_cast<int>(msg->width) != width_ || static_cast<int>(msg->height) != height_)
    {warn("image dimensions do not match CameraInfo/fallback calibration"); return;}
    MotionSample motion;
    if (mode_ == OrientationMode::ImuAided) {
      const double midpoint = previous_image_stamp_ ? 0.5 * (stamp + *previous_image_stamp_) : stamp;
      const auto sample = imu_.sample(midpoint + imu_camera_time_offset_, sensor_age_, interpolateImu);
      if (!sample) {odometry_->reset(); previous_image_stamp_.reset(); warn("missing time-aligned IMU"); return;}
      motion = motionSample(*sample);
      if (previous_image_stamp_ && stamp > *previous_image_stamp_) {
        const auto mean = imu_.meanVector(*previous_image_stamp_ + imu_camera_time_offset_,
          stamp + imu_camera_time_offset_, sensor_age_,
          [](const ImuSample & value) {return value.gyro;});
        if (!mean) {warn("camera interval not bracketed by IMU; using midpoint gyro");}
        else {motion.angular_velocity_body = *mean;}
      }
    }
    if (dynamic_height_) {
      const double midpoint=previous_image_stamp_ ? .5*(stamp+*previous_image_stamp_) : stamp;
      const auto height=heights_.sample(midpoint,sensor_age_,[](double a,double b,double t) {return a*(1-t)+b*t;});
      if (!height) {odometry_->reset(); previous_image_stamp_.reset(); warn("missing measured barometric height"); return;}
      motion.optical_height=*height+(rotationY(motion.pitch)*rotationX(motion.roll)*camera_offset_)[2];
      if (motion.optical_height < 2 || motion.optical_height > 500) {warn("optical height outside flight bounds"); return;}
    }
    try {
      const auto converted = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
      cv::Mat rectified;
      if (!distortion_.empty() && std::any_of(distortion_.begin(), distortion_.end(), [](double d) {return d != 0;})) {
        cv::undistort(converted->image, rectified, cv::Mat(camera_.matrix()), distortion_);
      } else {rectified = converted->image;}
      const auto result = odometry_->process(rectified, stamp, camera_, motion);
      previous_image_stamp_ = stamp;
      RCLCPP_INFO_THROTTLE(get_logger(), diagnostic_clock_, 5000,
        "Flow tracks=%zu inliers=%zu valid=%d velocity=(%.4f,%.4f,%.4f)",
        static_cast<size_t>(result.tracked), static_cast<size_t>(result.inliers), result.valid,
        result.velocity_body[0], result.velocity_body[1], result.velocity_body[2]);
      if (!result.valid) {warn(result.reason); return;}
      geometry_msgs::msg::TwistWithCovarianceStamped output;
      output.header.stamp = rosStamp(result.stamp); output.header.frame_id = body_frame_;
      output.twist.twist.linear.x = result.velocity_body[0]; output.twist.twist.linear.y = result.velocity_body[1];
      output.twist.twist.linear.z = result.velocity_body[2];
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {output.twist.covariance[r * 6 + c] = result.covariance_body(r, c);}
        output.twist.covariance[(r + 3) * 6 + r + 3] = 1e6;
      }
      publisher_->publish(output);
      if (!calibrated_) {warn("using configured fallback K until valid CameraInfo arrives");}
    } catch (const std::exception & e) {warn(e.what());}
  }
  void warn(const std::string & message)
  {RCLCPP_WARN_THROTTLE(get_logger(), diagnostic_clock_, 3000, "%s", message.c_str());}
  rclcpp::Clock diagnostic_clock_{RCL_STEADY_TIME};
  OrientationMode mode_;
  CameraModel camera_;
  std::vector<double> distortion_;
  int width_, height_;
  bool calibrated_{false};
  bool dynamic_height_{false};
  cv::Vec3d camera_offset_;
  TimeBuffer<double> heights_;
  double sensor_age_;
  double imu_camera_time_offset_;
  std::string body_frame_, optical_frame_;
  std::optional<double> previous_image_stamp_;
  TimeBuffer<ImuSample> imu_;
  std::unique_ptr<VisualOdometry> odometry_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr height_sub_;
};
rclcpp::Node::SharedPtr makeOpticalFlowNode(const rclcpp::NodeOptions & options)
{return std::make_shared<OpticalFlowNode>(options);}
}  // namespace glider_visual_nav
#ifndef GLIDER_VISUAL_NAV_NODE_LIBRARY
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {rclcpp::spin(std::make_shared<glider_visual_nav::OpticalFlowNode>());}
  catch (const std::exception & e) {RCLCPP_FATAL(rclcpp::get_logger("optical_flow_node"), "%s", e.what()); status = 1;}
  rclcpp::shutdown(); return status;
}
#endif
