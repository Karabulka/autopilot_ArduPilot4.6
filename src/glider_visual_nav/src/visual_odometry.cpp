#include "glider_visual_nav/visual_odometry.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace glider_visual_nav
{
cv::Matx33d px4Attitude(const cv::Matx33d & world_from_body)
{return enuToNed() * world_from_body * fluToFrd();}
namespace
{
constexpr double pi = 3.14159265358979323846;
bool finite(const cv::Vec3d & v)
{return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);}
bool finite(const cv::Matx33d & m)
{
  for (double value : m.val) {if (!std::isfinite(value)) {return false;}}
  return true;
}
cv::Matx33d matrix(const cv::Mat & m)
{
  cv::Matx33d out;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {out(r, c) = m.at<double>(r, c);}
  }
  return out;
}
}  // namespace
OrientationMode orientationMode(const std::string & value)
{
  if (value == "homography_mag") {return OrientationMode::HomographyMag;}
  if (value == "imu_aided") {return OrientationMode::ImuAided;}
  throw std::invalid_argument("orientation_mode must be homography_mag or imu_aided");
}
double wrapAngle(double a) {return std::atan2(std::sin(a), std::cos(a));}
cv::Matx33d rotationX(double a)
{return {1, 0, 0, 0, std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a)};}
cv::Matx33d rotationY(double a)
{return {std::cos(a), 0, std::sin(a), 0, 1, 0, -std::sin(a), 0, std::cos(a)};}
cv::Matx33d rotationZ(double a)
{return {std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a), 0, 0, 0, 1};}
cv::Matx33d opticalToBody() {return {0, -1, 0, -1, 0, 0, 0, 0, -1};}
cv::Matx33d fluToFrd() {return {1, 0, 0, 0, -1, 0, 0, 0, -1};}
cv::Matx33d enuToNed() {return {0, 1, 0, 1, 0, 0, 0, 0, -1};}
void CameraModel::validate() const
{
  if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 0 || fy <= 0 ||
    !std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(skew))
  {throw std::invalid_argument("invalid camera intrinsics");}
}
cv::Matx33d CameraModel::matrix() const
{validate(); return {fx, skew, cx, 0, fy, cy, 0, 0, 1};}
cv::Vec3d CameraModel::normalize(const cv::Point2d & p) const
{
  validate();
  const double y = (p.y - cy) / fy;
  return {(p.x - cx - skew * y) / fx, y, 1.0};
}
cv::Vec3d pixelVelocity(const CameraModel & k, double h, double dt, double du, double dv)
{
  k.validate();
  if (!std::isfinite(h) || h <= 0 || !std::isfinite(dt) || dt <= 0 ||
    !std::isfinite(du) || !std::isfinite(dv)) {throw std::invalid_argument("invalid pixel velocity input");}
  return opticalToBody() * (-h / dt * (k.matrix().inv() * cv::Vec3d(du, dv, 0.0)));
}
double magneticYaw(const cv::Vec3d & m, double roll, double pitch, double declination)
{
  if (!finite(m) || !std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(declination))
  {throw std::invalid_argument("invalid magnetic input");}
  const auto level = rotationY(pitch) * rotationX(roll) * m;
  if (std::hypot(level[0], level[1]) < 1e-9) {throw std::invalid_argument("horizontal magnetic field too small");}
  return wrapAngle(pi / 2.0 - declination - std::atan2(level[1], level[0]));
}
void FlowConfig::validate() const
{
  if (!std::isfinite(height) || height <= 0 || max_features < min_inliers || min_inliers < 4 ||
    !std::isfinite(feature_quality) || feature_quality <= 0 || feature_quality >= 1 ||
    !std::isfinite(feature_distance) || feature_distance <= 0 ||
    !std::isfinite(fb_threshold) || fb_threshold <= 0 ||
    !std::isfinite(ransac_threshold) || ransac_threshold <= 0 ||
    !std::isfinite(max_lk_error) || max_lk_error <= 0 || !std::isfinite(max_dt) || max_dt <= 0 ||
    !std::isfinite(velocity_noise_floor) || velocity_noise_floor <= 0 ||
    !std::isfinite(horizontal_tolerance) || horizontal_tolerance <= 0 || !finite(camera_offset_body))
  {throw std::invalid_argument("invalid flow configuration");}
}
FlowResult estimateTrackedVelocity(
  const std::vector<cv::Point2f> & previous, const std::vector<cv::Point2f> & current,
  const CameraModel & camera, const FlowConfig & config, double dt, const MotionSample & motion)
{
  config.validate(); camera.validate();
  const double height = motion.optical_height > 0 ? motion.optical_height : config.height;
  FlowResult out;
  out.dt = dt; out.tracked = static_cast<int>(previous.size());
  if (!std::isfinite(dt) || dt <= 1e-6 || dt > config.max_dt || previous.size() != current.size() ||
    previous.size() < static_cast<size_t>(config.min_inliers))
  {out.reason = "invalid interval or too few tracks"; return out;}
  for (size_t i = 0; i < previous.size(); ++i) {
    if (!std::isfinite(previous[i].x) || !std::isfinite(previous[i].y) ||
      !std::isfinite(current[i].x) || !std::isfinite(current[i].y))
    {out.reason = "non-finite track"; return out;}
  }
  cv::Mat mask;
  const auto homography = cv::findHomography(previous, current, cv::RANSAC, config.ransac_threshold, mask, 2000, 0.995);
  if (homography.empty()) {out.reason = "degenerate planar homography"; return out;}
  out.inliers = cv::countNonZero(mask);
  if (out.inliers < config.min_inliers) {out.reason = "too few RANSAC inliers"; return out;}
  const auto rbc = opticalToBody();
  const double noise_floor = config.velocity_noise_floor * config.velocity_noise_floor;
  auto cov_camera = cv::Matx33d::eye() * noise_floor;
  if (config.mode == OrientationMode::HomographyMag) {
    // With a horizontal plane and fixed height: Hn = R21 + t21*n1^T/h, n1=(0,0,1).
    auto hn = camera.matrix().inv() * matrix(homography) * camera.matrix();
    if (std::abs(hn(2, 2)) < 1e-9) {out.reason = "singular homography"; return out;}
    hn *= 1.0 / hn(2, 2);
    const double angle = std::atan2(hn(1, 0) - hn(0, 1), hn(0, 0) + hn(1, 1));
    const auto rot = rotationZ(angle);
    double discrepancy = 0;
    for (int r = 0; r < 2; ++r) {
      for (int c = 0; c < 2; ++c) {discrepancy = std::max(discrepancy, std::abs(hn(r, c) - rot(r, c)));}
    }
    if (!finite(hn) || discrepancy > config.horizontal_tolerance ||
      std::hypot(hn(2, 0), hn(2, 1)) > config.horizontal_tolerance)
    {out.reason = "homography violates horizontal fixed-height assumption"; return out;}
    const cv::Vec3d vc(-config.height * hn(0, 2) / dt, -config.height * hn(1, 2) / dt, 0);
    // Correct finite displacement of an offset camera (current-frame expression).
    const auto body_increment = rbc * rot * rbc.t();
    out.velocity_body = rbc * rotationZ(-0.5 * angle) * rbc.t() *
      (rbc * vc + (body_increment * config.camera_offset_body - config.camera_offset_body) / dt);
    double residual = 0;
    for (size_t i = 0; i < previous.size(); ++i) {
      if (!mask.at<unsigned char>(static_cast<int>(i))) {continue;}
      const auto a = camera.normalize(previous[i]), b = camera.normalize(current[i]);
      const auto predicted = rot * a + cv::Vec3d(hn(0, 2), hn(1, 2), 0);
      residual += std::pow(b[0] - predicted[0], 2) + std::pow(b[1] - predicted[1], 2);
    }
    const double variance = std::max(noise_floor, residual * config.height * config.height /
      (dt * dt * std::max(1, 2 * out.inliers - 3)));
    cov_camera(0, 0) = variance; cov_camera(1, 1) = variance;
    cov_camera(2, 2) = 1e6;  // vertical motion is unobserved in this constrained mode
  } else {
    if (!std::isfinite(motion.roll) || !std::isfinite(motion.pitch) || !finite(motion.angular_velocity_body))
    {out.reason = "invalid IMU motion"; return out;}
    const auto rwc = rotationY(motion.pitch) * rotationX(motion.roll) * rbc;
    const auto normal = -(rwc.t() * cv::Vec3d(0, 0, 1));
    const auto omega = rbc.t() * motion.angular_velocity_body;
    cv::Mat a(2 * out.inliers, 3, CV_64F), b(2 * out.inliers, 1, CV_64F);
    int row = 0;
    for (size_t i = 0; i < previous.size(); ++i) {
      if (!mask.at<unsigned char>(static_cast<int>(i))) {continue;}
      const auto q = camera.normalize(previous[i]), q2 = camera.normalize(current[i]);
      const double iz = normal.dot(q) / height;
      if (iz <= 1e-8) {out.reason = "ground behind camera"; return out;}
      const double x = q[0], y = q[1];
      const cv::Matx<double, 2, 3> rotational(x * y, -(1 + x * x), y, 1 + y * y, -x * y, -x);
      const auto spin = rotational * omega;
      a.at<double>(row, 0) = -iz; a.at<double>(row, 1) = 0; a.at<double>(row, 2) = x * iz;
      a.at<double>(row + 1, 0) = 0; a.at<double>(row + 1, 1) = -iz; a.at<double>(row + 1, 2) = y * iz;
      b.at<double>(row) = (q2[0] - x) / dt - spin[0];
      b.at<double>(row + 1) = (q2[1] - y) / dt - spin[1]; row += 2;
    }
    cv::SVD svd(a, cv::SVD::NO_UV);
    if (svd.w.at<double>(2) < 1e-8 || svd.w.at<double>(0) / svd.w.at<double>(2) > 1e6)
    {out.reason = "ill-conditioned flow geometry"; return out;}
    cv::Mat estimate;
    if (!cv::solve(a, b, estimate, cv::DECOMP_SVD)) {out.reason = "flow solve failed"; return out;}
    // Huber IRLS: one weight per feature, shared by both image components.
    for (int iteration = 0; iteration < 3; ++iteration) {
      const cv::Mat residual = a * estimate - b;
      std::vector<double> norms;
      norms.reserve(row / 2);
      for (int j = 0; j < row; j += 2) {
        norms.push_back(std::hypot(residual.at<double>(j), residual.at<double>(j + 1)));
      }
      auto middle = norms.begin() + norms.size() / 2;
      std::nth_element(norms.begin(), middle, norms.end());
      const double cutoff = std::max(1e-5, 1.5 * *middle);
      cv::Mat weighted_a = a.clone(), weighted_b = b.clone();
      for (int j = 0; j < row; j += 2) {
        const double norm = std::hypot(residual.at<double>(j), residual.at<double>(j + 1));
        const double weight = std::sqrt(cutoff / std::max(cutoff, norm));
        weighted_a.row(j) *= weight;
        weighted_a.row(j + 1) *= weight;
        weighted_b.at<double>(j) *= weight;
        weighted_b.at<double>(j + 1) *= weight;
      }
      if (!cv::solve(weighted_a, weighted_b, estimate, cv::DECOMP_SVD)) {
        out.reason = "weighted flow solve failed"; return out;
      }
    }
    const cv::Vec3d vc(estimate.at<double>(0), estimate.at<double>(1), estimate.at<double>(2));
    const cv::Mat residual = a * estimate - b;
    const double variance = residual.dot(residual) / std::max(1, row - 3);
    cv::Mat info = a.t() * a, inverse_info;
    cv::invert(info, inverse_info, cv::DECOMP_SVD);
    cov_camera = matrix(inverse_info) * variance + cv::Matx33d::eye() * noise_floor;
    out.velocity_body = rbc * vc - motion.angular_velocity_body.cross(config.camera_offset_body);
  }
  out.covariance_body = rbc * cov_camera * rbc.t();
  out.valid = finite(out.velocity_body) && finite(out.covariance_body);
  out.reason = out.valid ? "ok" : "non-finite velocity";
  return out;
}
VisualOdometry::VisualOdometry(const FlowConfig & config) : config_(config) {config_.validate();}
void VisualOdometry::reset() {previous_.release(); previous_stamp_ = 0;}
FlowResult VisualOdometry::process(const cv::Mat & mono, double stamp, const CameraModel & camera, const MotionSample & motion)
{
  FlowResult out;
  if (mono.empty() || mono.type() != CV_8UC1 || !std::isfinite(stamp))
  {out.reason = "expected finite timestamp and MONO8 image"; return out;}
  const auto k = camera.matrix();
  const double dt = stamp - previous_stamp_;
  if (previous_.empty() || previous_.size() != mono.size() || cv::norm(k - previous_k_) > 1e-9 ||
    dt <= 1e-6 || dt > config_.max_dt)
  {
    if (!previous_.empty() && dt == 0) {out.reason = "duplicate image"; return out;}
    previous_ = mono.clone(); previous_stamp_ = stamp; previous_k_ = k;
    out.reason = "initializing or resetting image pair"; return out;
  }
  try {
    std::vector<cv::Point2f> source, target, back, good_source, good_target;
    std::vector<unsigned char> forward_status, backward_status;
    std::vector<float> forward_error, backward_error;
    cv::goodFeaturesToTrack(previous_, source, config_.max_features, config_.feature_quality, config_.feature_distance);
    if (!source.empty()) {
      if (mono.cols >= 11 && mono.rows >= 11) {
        cv::cornerSubPix(previous_, source, cv::Size(3, 3), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.01));
      }
      cv::calcOpticalFlowPyrLK(previous_, mono, source, target, forward_status, forward_error);
      cv::calcOpticalFlowPyrLK(mono, previous_, target, back, backward_status, backward_error);
      for (size_t i = 0; i < source.size(); ++i) {
        if (forward_status[i] && backward_status[i] && forward_error[i] <= config_.max_lk_error &&
          cv::norm(source[i] - back[i]) <= config_.fb_threshold && target[i].x >= 0 && target[i].y >= 0 &&
          target[i].x < mono.cols && target[i].y < mono.rows)
        {good_source.push_back(source[i]); good_target.push_back(target[i]);}
      }
    }
    out = estimateTrackedVelocity(good_source, good_target, camera, config_, dt, motion);
    out.stamp = 0.5 * (stamp + previous_stamp_);
  } catch (const cv::Exception & e) {out.reason = e.what();}
  previous_ = mono.clone(); previous_stamp_ = stamp; previous_k_ = k;
  return out;
}
DeadReckoner::DeadReckoner(double height, double max_dt, double yaw_variance)
: max_dt_(max_dt), yaw_variance_(yaw_variance)
{
  if (!std::isfinite(height) || height <= 0 || !std::isfinite(max_dt) || max_dt <= 0 ||
    !std::isfinite(yaw_variance) || yaw_variance < 0) {throw std::invalid_argument("invalid navigation configuration");}
  state_.position[2] = height;
}
bool DeadReckoner::update(double stamp, const cv::Vec3d & velocity, const cv::Matx33d & covariance,
  double yaw, double roll, double pitch)
{
  if (!std::isfinite(stamp) || !finite(velocity) || !finite(covariance) || !std::isfinite(yaw) ||
    !std::isfinite(roll) || !std::isfinite(pitch)) {return false;}
  if (initialized_ && stamp <= previous_stamp_) {return false;}
  const auto rot = rotationZ(yaw) * rotationY(pitch) * rotationX(roll);
  const auto world = rot * velocity;
  const cv::Vec3d j(-world[1], world[0], 0);
  const cv::Matx33d cov_world = rot * covariance * rot.t() + yaw_variance_ * (j * j.t());
  state_.discontinuity = initialized_ && stamp - previous_stamp_ > max_dt_;
  if (initialized_ && !state_.discontinuity) {
    const double dt = stamp - previous_stamp_;
    const auto delta = 0.5 * dt * (previous_velocity_ + world);
    state_.position[0] += delta[0]; state_.position[1] += delta[1];
    // Local independent-sample approximation, not a correlated visual-inertial EKF.
    state_.position_covariance += 0.5 * dt * dt * (previous_covariance_ + cov_world);
  } else if (state_.discontinuity) {
    const double gap = stamp - previous_stamp_;
    const double unknown = gap * gap * std::max(1.0, world.dot(world));
    state_.position_covariance(0, 0) += unknown; state_.position_covariance(1, 1) += unknown;
  }
  state_.velocity_world = world; previous_velocity_ = world; previous_covariance_ = cov_world;
  previous_stamp_ = stamp; initialized_ = true;
  return true;
}
void DeadReckoner::reset()
{
  const double height = state_.position[2];
  state_ = NavigationState{};
  state_.position[2] = height;
  initialized_ = false;
}
LosResult lineOfSight(const cv::Point2d & a, const cv::Point2d & b, const cv::Point2d & p,
  double course, double speed, double lookahead, double max_roll, double gravity)
{
  const auto delta = b - a;
  const double length = cv::norm(delta);
  if (!std::isfinite(length) || length < 1e-6 || !std::isfinite(p.x) || !std::isfinite(p.y) ||
    !std::isfinite(course) || !std::isfinite(speed) || speed < 0 || !std::isfinite(lookahead) || lookahead <= 0 ||
    !std::isfinite(max_roll) || max_roll <= 0 || max_roll >= pi / 2 || !std::isfinite(gravity) || gravity <= 0)
  {throw std::invalid_argument("invalid LOS input");}
  const auto relative = p - a;
  LosResult out;
  out.cross_track = (-delta.y * relative.x + delta.x * relative.y) / length;
  out.along_track = delta.dot(relative) / length;
  out.desired_course = wrapAngle(std::atan2(delta.y, delta.x) - std::atan2(out.cross_track, lookahead));
  const double limit = gravity * std::tan(max_roll);
  const double acceleration = std::clamp(2 * speed * speed / lookahead *
    std::sin(wrapAngle(out.desired_course - course)), -limit, limit);
  out.desired_roll = -std::atan2(acceleration, gravity);
  return out;
}
HeightPid::HeightPid(const PidConfig & config) : config_(config)
{
  if (!std::isfinite(config.kp) || !std::isfinite(config.ki) || !std::isfinite(config.kd) ||
    config.kp < 0 || config.ki < 0 || config.kd < 0 || !std::isfinite(config.min_output) ||
    !std::isfinite(config.max_output) || config.min_output >= config.max_output ||
    !std::isfinite(config.integral_limit) || config.integral_limit < 0)
  {throw std::invalid_argument("invalid PID configuration");}
}
double HeightPid::update(double error, double vertical_speed, double dt)
{
  if (!std::isfinite(error) || !std::isfinite(vertical_speed) || !std::isfinite(dt) || dt <= 0)
  {throw std::invalid_argument("invalid PID sample");}
  const double candidate = std::clamp(integral_ + error * dt, -config_.integral_limit, config_.integral_limit);
  const double raw = config_.kp * error + config_.ki * candidate - config_.kd * vertical_speed;
  if ((raw <= config_.max_output || error < 0) && (raw >= config_.min_output || error > 0)) {integral_ = candidate;}
  return std::clamp(config_.kp * error + config_.ki * integral_ - config_.kd * vertical_speed,
    config_.min_output, config_.max_output);
}
}  // namespace glider_visual_nav
