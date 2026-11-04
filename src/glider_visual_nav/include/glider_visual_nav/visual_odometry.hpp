#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace glider_visual_nav
{
enum class OrientationMode {HomographyMag, ImuAided};
OrientationMode orientationMode(const std::string & value);
double wrapAngle(double angle);
cv::Matx33d rotationX(double angle);
cv::Matx33d rotationY(double angle);
cv::Matx33d rotationZ(double angle);
cv::Matx33d opticalToBody();
cv::Matx33d fluToFrd();
cv::Matx33d enuToNed();
// Desired FRD->NED rotation, exact proper rotations on both sides.
cv::Matx33d px4Attitude(const cv::Matx33d & world_from_body);
struct CameraModel
{
  double fx{539.936}, fy{539.936}, cx{640.0}, cy{480.0}, skew{0.0};
  cv::Matx33d matrix() const;
  cv::Vec3d normalize(const cv::Point2d & pixel) const;
  void validate() const;
};
// Level, non-rotating camera; flow is current minus previous pixels.
cv::Vec3d pixelVelocity(const CameraModel & camera, double height, double dt, double du, double dv);
// Calibrated FLU field in tesla; pitch is FLU Euler pitch, declination east-positive.
double magneticYaw(const cv::Vec3d & field_body, double roll, double pitch, double declination);
struct MotionSample
{
  double roll{0.0}, pitch{0.0};
  cv::Vec3d angular_velocity_body{0.0, 0.0, 0.0};
  double optical_height{0.0};  // <=0 keeps configured constant height
};
struct FlowConfig
{
  double height{40.0};
  OrientationMode mode{OrientationMode::HomographyMag};
  int max_features{300}, min_inliers{12};
  double feature_quality{0.01}, feature_distance{8.0};
  double fb_threshold{0.7}, ransac_threshold{1.5}, max_lk_error{30.0};
  double max_dt{0.25}, velocity_noise_floor{0.02}, horizontal_tolerance{0.05};
  cv::Vec3d camera_offset_body{0.0, 0.0, 0.0};
  void validate() const;
};
struct FlowResult
{
  bool valid{false};
  std::string reason;
  cv::Vec3d velocity_body{0.0, 0.0, 0.0};
  cv::Matx33d covariance_body{cv::Matx33d::eye()};
  double dt{0.0}, stamp{0.0};  // interval average; timestamp at interval midpoint
  int tracked{0}, inliers{0};
};
FlowResult estimateTrackedVelocity(
  const std::vector<cv::Point2f> & previous, const std::vector<cv::Point2f> & current,
  const CameraModel & camera, const FlowConfig & config, double dt,
  const MotionSample & motion = MotionSample{});
class VisualOdometry
{
public:
  explicit VisualOdometry(const FlowConfig & config);
  FlowResult process(const cv::Mat & mono, double stamp, const CameraModel & camera,
    const MotionSample & motion = MotionSample{});
  void reset();
private:
  FlowConfig config_;
  cv::Mat previous_;
  cv::Matx33d previous_k_{cv::Matx33d::zeros()};
  double previous_stamp_{0.0};
};
struct NavigationState
{
  cv::Vec3d position{0.0, 0.0, 40.0};
  cv::Matx33d position_covariance{cv::Matx33d::zeros()};
  cv::Vec3d velocity_world{0.0, 0.0, 0.0};
  bool discontinuity{false};
};
class DeadReckoner
{
public:
  DeadReckoner(double height, double max_dt, double yaw_variance);
  bool update(double stamp, const cv::Vec3d & velocity_body, const cv::Matx33d & covariance_body,
    double yaw, double roll = 0.0, double pitch = 0.0);
  const NavigationState & state() const {return state_;}
  void reset();
private:
  NavigationState state_;
  bool initialized_{false};
  double previous_stamp_{0.0}, max_dt_, yaw_variance_;
  cv::Vec3d previous_velocity_{0.0, 0.0, 0.0};
  cv::Matx33d previous_covariance_{cv::Matx33d::zeros()};
};
struct LosResult
{
  double cross_track{0.0}, along_track{0.0}, desired_course{0.0}, desired_roll{0.0};
};
LosResult lineOfSight(const cv::Point2d & start, const cv::Point2d & end, const cv::Point2d & position,
  double course, double speed, double lookahead, double max_roll, double gravity = 9.80665);
struct PidConfig
{
  double kp{0.02}, ki{0.001}, kd{0.03};
  double min_output{-0.15}, max_output{0.15}, integral_limit{20.0};
};
class HeightPid
{
public:
  explicit HeightPid(const PidConfig & config);
  double update(double error, double vertical_speed, double dt);
  void reset() {integral_ = 0.0;}
private:
  PidConfig config_;
  double integral_{0.0};
};
}  // namespace glider_visual_nav
