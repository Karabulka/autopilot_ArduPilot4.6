#include "glider_visual_nav/visual_odometry.hpp"
#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <limits>

namespace gv = glider_visual_nav;
namespace
{
constexpr double pi = 3.14159265358979323846;
gv::CameraModel camera() {return {540, 540, 640, 480, 0};}
std::vector<cv::Point2f> grid()
{
  std::vector<cv::Point2f> result;
  for (int y = 200; y <= 700; y += 100) {
    for (int x = 300; x <= 1000; x += 100) {result.emplace_back(x, y);}
  }
  return result;
}
std::vector<cv::Point2f> warp(const std::vector<cv::Point2f> & points, const cv::Matx33d & h)
{
  std::vector<cv::Point2f> result;
  for (const auto & p : points) {
    const auto q = h * cv::Vec3d(p.x, p.y, 1);
    result.emplace_back(q[0] / q[2], q[1] / q[2]);
  }
  return result;
}
}  // namespace

TEST(PixelVelocity, RequiredMetricExample)
{
  const auto v = gv::pixelVelocity(camera(), 40, 1.0 / 30, 0.9, 5.4);
  EXPECT_NEAR(v[0], 12, 1e-12); EXPECT_NEAR(v[1], 2, 1e-12); EXPECT_DOUBLE_EQ(v[2], 0);
}
TEST(Camera, InverseIntrinsicsIncludesSkew)
{
  const gv::CameraModel k{510, 530, 310, 240, 17};
  const auto q = k.normalize({400, 300});
  const auto expected = k.matrix().inv() * cv::Vec3d(400, 300, 1);
  EXPECT_LT(cv::norm(q - expected), 1e-12);
  EXPECT_THROW(gv::pixelVelocity(k, 0, 0.1, 1, 1), std::invalid_argument);
  EXPECT_THROW(gv::pixelVelocity(k, 40, 0, 1, 1), std::invalid_argument);
  auto bad = k; bad.fx = -1;
  EXPECT_THROW(bad.validate(), std::invalid_argument);
}
TEST(Frames, ProperOrthogonalRotationsAndPx4Conversion)
{
  for (const auto & r : {gv::opticalToBody(), gv::fluToFrd(), gv::enuToNed(),
    gv::rotationX(0.3), gv::rotationY(-0.2), gv::rotationZ(0.8)})
  {
    EXPECT_NEAR(cv::determinant(cv::Mat(r)), 1.0, 1e-12);
    EXPECT_LT(cv::norm(r * r.t() - cv::Matx33d::eye()), 1e-12);
  }
  const auto r = gv::rotationZ(0.8) * gv::rotationY(-0.2) * gv::rotationX(0.3);
  const auto converted = gv::enuToNed() * r * gv::fluToFrd();
  const auto expected = gv::rotationZ(pi / 2 - 0.8) * gv::rotationY(0.2) * gv::rotationX(0.3);
  EXPECT_LT(cv::norm(converted - expected), 1e-12);
}
TEST(Magnetometer, RestoresYawWithTiltAndDeclination)
{
  const double roll = 0.31, pitch = -0.23, declination = 0.12;
  const cv::Vec3d world(23e-6 * std::sin(declination), 23e-6 * std::cos(declination), -42e-6);
  for (double yaw : {-3.13, -1.4, 0.0, 0.8, 3.13}) {
    const auto r = gv::rotationZ(yaw) * gv::rotationY(pitch) * gv::rotationX(roll);
    EXPECT_NEAR(gv::wrapAngle(gv::magneticYaw(r.t() * world, roll, pitch, declination) - yaw), 0, 1e-12);
  }
  EXPECT_THROW(gv::magneticYaw({0, 0, 0}, 0, 0, 0), std::invalid_argument);
  EXPECT_THROW(gv::magneticYaw({0, 0, 40e-6}, 0, 0, 0), std::invalid_argument);
}
TEST(Homography, MetricTranslationWithRansacOutliers)
{
  const auto previous = grid(); auto current = previous;
  for (auto & p : current) {p += cv::Point2f(0.9F, 5.4F);}
  for (size_t i = 0; i < 10; ++i) {current[i] += cv::Point2f(40 + i * 3, -30 - i * 7);}
  const auto result = gv::estimateTrackedVelocity(previous, current, camera(), gv::FlowConfig{}, 1.0 / 30);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_NEAR(result.velocity_body[0], 12, 1e-3); EXPECT_NEAR(result.velocity_body[1], 2, 1e-3);
  EXPECT_LT(result.inliers, static_cast<int>(previous.size()));
  EXPECT_GT(result.covariance_body(0, 0), 0);
}
TEST(Homography, PureYawIsNotTranslation)
{
  const auto previous = grid(); const auto k = camera().matrix();
  const auto current = warp(previous, k * gv::rotationZ(0.05) * k.inv());
  const auto result = gv::estimateTrackedVelocity(previous, current, camera(), gv::FlowConfig{}, 1.0 / 30);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_LT(cv::norm(result.velocity_body), 1e-3);
}
TEST(Homography, RotatingOffsetCameraDoesNotMoveBodyOrigin)
{
  gv::FlowConfig config; config.camera_offset_body = {0.5, 0.2, 0};
  const auto rbc = gv::opticalToBody(), rot = gv::rotationZ(0.05);
  const auto body_increment = rbc * rot * rbc.t();
  const auto displacement_camera = rbc.t() * (config.camera_offset_body - body_increment * config.camera_offset_body);
  auto hn = rot;
  hn(0, 2) = -displacement_camera[0] / config.height;
  hn(1, 2) = -displacement_camera[1] / config.height;
  const auto previous = grid(); const auto k = camera().matrix();
  const auto result = gv::estimateTrackedVelocity(previous, warp(previous, k * hn * k.inv()), camera(), config, 1.0 / 30);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_LT(cv::norm(result.velocity_body), 1e-3);
}
TEST(Homography, SimultaneousYawAndTranslationUsesMidpointFrame)
{
  const double angle = 0.04, dt = 1.0 / 30;
  const auto rbc = gv::opticalToBody(), k = camera().matrix();
  const cv::Vec3d desired_body(12, 2, 0);
  const auto camera_current = gv::rotationZ(angle / 2) * rbc.t() * desired_body;
  auto hn = gv::rotationZ(angle);
  hn(0, 2) = -camera_current[0] * dt / 40; hn(1, 2) = -camera_current[1] * dt / 40;
  const auto previous = grid();
  const auto result = gv::estimateTrackedVelocity(previous, warp(previous, k * hn * k.inv()), camera(), gv::FlowConfig{}, dt);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_LT(cv::norm(result.velocity_body - desired_body), 1e-3);
}
TEST(Homography, RejectsScaleChange)
{
  const auto previous = grid(); const auto k = camera().matrix();
  const cv::Matx33d scale(1.2, 0, 0, 0, 1.2, 0, 0, 0, 1);
  EXPECT_FALSE(gv::estimateTrackedVelocity(previous, warp(previous, k * scale * k.inv()),
    camera(), gv::FlowConfig{}, 1.0 / 30).valid);
}
TEST(ImuAided, RemovesRotationalFlowAndTiltWithLeverArm)
{
  gv::FlowConfig config; config.mode = gv::OrientationMode::ImuAided; config.camera_offset_body = {0.3, 0.1, 0.2};
  gv::MotionSample motion; motion.roll = 0.12; motion.pitch = -0.08;
  motion.angular_velocity_body = {0.03, -0.04, 0.02};
  const auto rbc = gv::opticalToBody();
  const auto normal = -((gv::rotationY(motion.pitch) * gv::rotationX(motion.roll) * rbc).t() * cv::Vec3d(0, 0, 1));
  const cv::Vec3d desired_body(12, 2, 0.4);
  const auto vc = rbc.t() * (desired_body + motion.angular_velocity_body.cross(config.camera_offset_body));
  const auto omega = rbc.t() * motion.angular_velocity_body;
  const auto previous = grid(); std::vector<cv::Point2f> current;
  constexpr double dt = 0.001;
  for (const auto & p : previous) {
    const auto q = camera().normalize(p); const double x = q[0], y = q[1], iz = normal.dot(q) / 40;
    const cv::Matx<double, 2, 3> a(-iz, 0, x * iz, 0, -iz, y * iz);
    const cv::Matx<double, 2, 3> b(x * y, -(1 + x * x), y, 1 + y * y, -x * y, -x);
    const auto flow = a * vc + b * omega;
    current.emplace_back(p.x + 540 * dt * flow[0], p.y + 540 * dt * flow[1]);
  }
  const auto result = gv::estimateTrackedVelocity(previous, current, camera(), config, dt, motion);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_LT(cv::norm(result.velocity_body - desired_body), 0.002);
}
TEST(Tracking, LucasKanadeForwardBackwardOnSyntheticTexture)
{
  gv::CameraModel k{540, 540, 160, 120, 0};
  cv::Mat image(240, 320, CV_8UC1);
  cv::RNG rng(1234); rng.fill(image, cv::RNG::UNIFORM, 0, 256);
  cv::GaussianBlur(image, image, {5, 5}, 0.8);
  cv::Mat shifted;
  cv::warpAffine(image, shifted, cv::Mat(cv::Matx23d(1, 0, 0.9, 0, 1, 5.4)), image.size());
  gv::VisualOdometry vo(gv::FlowConfig{});
  EXPECT_FALSE(vo.process(image, 1.0, k).valid);
  const auto result = vo.process(shifted, 1.0 + 1.0 / 30, k);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_NEAR(result.velocity_body[0], 12, 0.2); EXPECT_NEAR(result.velocity_body[1], 2, 0.2);
  EXPECT_NEAR(result.stamp, 1.0 + 1.0 / 60, 1e-12);
}
TEST(Tracking, BlankImageAndTimeRewindDoNotPublishVelocity)
{
  gv::VisualOdometry vo(gv::FlowConfig{});
  cv::Mat blank = cv::Mat::zeros(240, 320, CV_8UC1);
  EXPECT_FALSE(vo.process(blank, 2, camera()).valid);
  EXPECT_FALSE(vo.process(blank, 2.03, camera()).valid);
  EXPECT_FALSE(vo.process(blank, 1, camera()).valid);
  EXPECT_FALSE(vo.process(blank, 1, camera()).valid);
  EXPECT_FALSE(gv::estimateTrackedVelocity({}, {}, camera(), gv::FlowConfig{}, 0).valid);
}
TEST(DeadReckoning, IntegratesTrapezoidsAndRotatesToEnu)
{
  gv::DeadReckoner nav(40, 0.25, 0.0025);
  ASSERT_TRUE(nav.update(0, {10, 0, 0}, cv::Matx33d::eye(), pi / 2));
  ASSERT_TRUE(nav.update(0.1, {20, 0, 0}, cv::Matx33d::eye(), pi / 2));
  EXPECT_NEAR(nav.state().position[0], 0, 1e-12);
  EXPECT_NEAR(nav.state().position[1], 1.5, 1e-12);
  EXPECT_DOUBLE_EQ(nav.state().position[2], 40);
  EXPECT_GT(nav.state().position_covariance(1, 1), 0);
}
TEST(DeadReckoning, GapAndRewindDoNotInventDistance)
{
  gv::DeadReckoner nav(40, 0.25, 0.0025);
  ASSERT_TRUE(nav.update(1, {10, 0, 0}, cv::Matx33d::eye(), 0));
  EXPECT_FALSE(nav.update(0.9, {10, 0, 0}, cv::Matx33d::eye(), 0));
  ASSERT_TRUE(nav.update(3, {10, 0, 0}, cv::Matx33d::eye(), 0));
  EXPECT_TRUE(nav.state().discontinuity); EXPECT_DOUBLE_EQ(nav.state().position[0], 0);
  EXPECT_GT(nav.state().position_covariance(0, 0), 100);
  nav.reset(); EXPECT_TRUE(nav.update(0, {10, 0, 0}, cv::Matx33d::eye(), 0));
}
TEST(Los, CorrectBankSignOnBothSidesOfEastboundLine)
{
  const auto left = gv::lineOfSight({0, 0}, {200, 0}, {50, 20}, 0, 12, 40, 0.52);
  const auto right = gv::lineOfSight({0, 0}, {200, 0}, {50, -20}, 0, 12, 40, 0.52);
  EXPECT_GT(left.cross_track, 0); EXPECT_LT(left.desired_course, 0); EXPECT_GT(left.desired_roll, 0);
  EXPECT_LT(right.cross_track, 0); EXPECT_GT(right.desired_course, 0); EXPECT_LT(right.desired_roll, 0);
  EXPECT_NEAR(left.desired_roll, -right.desired_roll, 1e-12);
  EXPECT_DOUBLE_EQ(gv::lineOfSight({0, 0}, {200, 0}, {50, 0}, 0, 12, 40, 0.52).desired_roll, 0);
}
TEST(Los, LimitsBankAndRejectsDegenerateSegments)
{
  const auto result = gv::lineOfSight({0, 0}, {200, 0}, {50, 500}, 0, 100, 10, 0.3);
  EXPECT_NEAR(result.desired_roll, 0.3, 1e-12);
  EXPECT_THROW(gv::lineOfSight({0, 0}, {0, 0}, {1, 1}, 0, 12, 40, 0.52), std::invalid_argument);
}
TEST(HeightPid, DampingSaturationAndAntiWindup)
{
  gv::PidConfig config; config.kp = 0.1; config.ki = 1; config.kd = 0.2;
  gv::HeightPid pid(config);
  for (int i = 0; i < 1000; ++i) {EXPECT_LE(pid.update(100, 0, 0.1), config.max_output);}
  EXPECT_LT(pid.update(-0.1, 0, 0.1), 0);
  pid.reset(); EXPECT_LT(pid.update(0, 1, 0.1), 0);
  EXPECT_THROW(pid.update(0, 0, 0), std::invalid_argument);
}
TEST(Configuration, RejectsUnknownModeAndInvalidParameters)
{
  EXPECT_EQ(gv::orientationMode("homography_mag"), gv::OrientationMode::HomographyMag);
  EXPECT_EQ(gv::orientationMode("imu_aided"), gv::OrientationMode::ImuAided);
  EXPECT_THROW(gv::orientationMode("unknown"), std::invalid_argument);
  gv::FlowConfig config; config.height = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(config.validate(), std::invalid_argument);
}
TEST(Px4Attitude, ExactFrameConversionPreservesPhysicalRollAndPitch)
{
  for(double yaw : {-2.2,0.,1.3}) {
    const double roll=.23,pitch=-.14;
    const auto wb=gv::rotationZ(yaw)*gv::rotationY(pitch)*gv::rotationX(roll);
    const auto nr=gv::px4Attitude(wb);
    const auto expected=gv::rotationZ(pi/2-yaw)*gv::rotationY(-pitch)*gv::rotationX(roll);
    EXPECT_NEAR(cv::norm(nr-expected),0,1e-12);
    EXPECT_NEAR(cv::determinant(cv::Mat(nr)),1,1e-12);
    EXPECT_NEAR(cv::norm(gv::enuToNed().t()*nr*gv::fluToFrd()-wb),0,1e-12);
  }
}
TEST(ImuAided, MeasuredHeightOverridesConstantScale)
{
  gv::FlowConfig config; config.mode=gv::OrientationMode::ImuAided;
  const auto previous=grid(); std::vector<cv::Point2f> current;
  for(const auto & p:previous) {current.emplace_back(p.x+.9,p.y+5.4);}
  gv::MotionSample motion; motion.optical_height=20;
  const auto result=gv::estimateTrackedVelocity(previous,current,camera(),config,1./30,motion);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_NEAR(result.velocity_body[0],6,.01);
  EXPECT_NEAR(result.velocity_body[1],1,.01);
}
