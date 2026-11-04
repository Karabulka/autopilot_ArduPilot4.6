#include "glider_visual_nav/nodes.hpp"
#include "glider_visual_nav/ros_support.hpp"
#include <gtest/gtest.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <thread>

namespace gv = glider_visual_nav;
using namespace std::chrono_literals;

TEST(RosNodes, BothModesFallbackCameraInfoOdometryGuidanceAndStaleSensors)
{
  int domain = 201;
  for (const std::string mode : {"homography_mag", "imu_aided"}) {
    SCOPED_TRACE(mode);
    auto context = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init; init.set_domain_id(domain++);
    context->init(0, nullptr, init);
    struct Shutdown {rclcpp::Context::SharedPtr ctx; ~Shutdown() {ctx->shutdown("test complete");}} shutdown{context};
    rclcpp::NodeOptions common;
    common.context(context).use_global_arguments(false).use_intra_process_comms(true);
    common.parameter_overrides({rclcpp::Parameter("use_sim_time", true), rclcpp::Parameter("orientation_mode", mode)});
    auto optical_options = common;
    auto overrides = optical_options.parameter_overrides();
    for (const auto & value : {
      rclcpp::Parameter("image_width", 320), rclcpp::Parameter("image_height", 240),
      rclcpp::Parameter("fx", 540.0), rclcpp::Parameter("fy", 540.0),
      rclcpp::Parameter("cx", 160.0), rclcpp::Parameter("cy", 120.0)}) {overrides.push_back(value);}
    optical_options.parameter_overrides(overrides);
    auto optical = gv::makeOpticalFlowNode(optical_options);
    auto navigation = gv::makeNavigationNode(common);
    auto guidance = gv::makeLosControllerNode(common);
    auto driver = std::make_shared<rclcpp::Node>("synthetic_sensors", common);
    rclcpp::ExecutorOptions executor_options; executor_options.context = context;
    rclcpp::executors::SingleThreadedExecutor executor(executor_options);
    for (const auto & node : {optical, navigation, guidance, driver}) {executor.add_node(node);}
    auto pump = [&executor](int cycles = 10) {
        for (int i = 0; i < cycles; ++i) {executor.spin_some(); std::this_thread::sleep_for(1ms);}
      };
    std::vector<geometry_msgs::msg::TwistWithCovarianceStamped> velocities;
    std::vector<nav_msgs::msg::Odometry> positions;
    std::vector<geometry_msgs::msg::PoseStamped> targets;
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    auto velocity_sub = driver->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "glider/visual_velocity", 10, [&velocities](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr m) {velocities.push_back(*m);});
    auto odometry_sub = driver->create_subscription<nav_msgs::msg::Odometry>("glider/odometry", 10,
      [&positions](nav_msgs::msg::Odometry::ConstSharedPtr m) {positions.push_back(*m);});
    auto target_sub = driver->create_subscription<geometry_msgs::msg::PoseStamped>("glider/target_pose", 10,
      [&targets](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {targets.push_back(*m);});
    auto tf_sub = driver->create_subscription<tf2_msgs::msg::TFMessage>("tf", 10,
      [&transforms](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {
        transforms.insert(transforms.end(), m->transforms.begin(), m->transforms.end());
      });
    auto images = driver->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", rclcpp::SensorDataQoS());
    auto infos = driver->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", rclcpp::SensorDataQoS());
    auto magnetics = driver->create_publisher<sensor_msgs::msg::MagneticField>("glider/magnetic_field", rclcpp::SensorDataQoS());
    auto imus = driver->create_publisher<sensor_msgs::msg::Imu>("glider/imu", rclcpp::SensorDataQoS());
    auto clock = driver->create_publisher<rosgraph_msgs::msg::Clock>("clock", rclcpp::ClockQoS());
    cv::Mat texture(240, 320, CV_8UC1);
    cv::RNG rng(42); rng.fill(texture, cv::RNG::UNIFORM, 0, 256);
    cv::GaussianBlur(texture, texture, {5, 5}, 0.8);
    auto frame = [&](int i, bool publish_mag) {
        const auto time = gv::rosStamp(1.0 + static_cast<double>(i) / 30);
        rosgraph_msgs::msg::Clock tick; tick.clock = time; clock->publish(tick);
        if (publish_mag) {
          sensor_msgs::msg::MagneticField field;
          field.header.stamp = time; field.header.frame_id = "base_link";
          field.magnetic_field.y = 23e-6; field.magnetic_field.z = -42e-6; magnetics->publish(field);
        }
        sensor_msgs::msg::Imu imu; imu.header.stamp = time; imu.header.frame_id = "base_link";
        imu.orientation.w = 1; imus->publish(imu);
        pump();
        cv::Mat shifted;
        cv::warpAffine(texture, shifted, cv::Mat(cv::Matx23d(1, 0, 0.45 * i, 0, 1, 2.7 * i)),
          texture.size(), cv::INTER_LINEAR, cv::BORDER_WRAP);
        sensor_msgs::msg::Image image;
        image.header.stamp = time; image.header.frame_id = "camera_optical_frame";
        image.width = 320; image.height = 240; image.encoding = "mono8"; image.step = 320;
        image.data.assign(shifted.datastart, shifted.dataend); images->publish(image);
        pump();
      };
    pump(100);
    for (int i = 0; i < 5; ++i) {frame(i, false);}
    ASSERT_GE(velocities.size(), 2U);
    EXPECT_TRUE(positions.empty()); EXPECT_TRUE(targets.empty());
    EXPECT_NEAR(velocities.back().twist.twist.linear.x, 6, 0.2);
    EXPECT_NEAR(velocities.back().twist.twist.linear.y, 1, 0.2);
    for (int i = 5; i < 13; ++i) {frame(i, true);}
    ASSERT_FALSE(positions.empty()); ASSERT_FALSE(targets.empty()); ASSERT_FALSE(transforms.empty());
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id = "camera_optical_frame"; info.width = 320; info.height = 240;
    info.distortion_model = "plumb_bob";
    info.k = {270, 0, 160, 0, 270, 120, 0, 0, 1}; infos->publish(info); pump();
    for (int i = 13; i < 25; ++i) {frame(i, true);}
    ASSERT_FALSE(velocities.empty());
    EXPECT_NEAR(velocities.back().twist.twist.linear.x, 12, 0.35);
    EXPECT_NEAR(velocities.back().twist.twist.linear.y, 2, 0.35);
    const auto & odom = positions.back();
    EXPECT_EQ(odom.header.frame_id, "odom"); EXPECT_EQ(odom.child_frame_id, "base_link");
    EXPECT_GT(odom.pose.pose.position.x, 0.5); EXPECT_GT(odom.pose.pose.position.y, 0);
    EXPECT_DOUBLE_EQ(odom.pose.pose.position.z, 40);
    EXPECT_NEAR(odom.pose.pose.orientation.z, 0, 1e-10);
    EXPECT_NEAR(odom.twist.twist.angular.z, 0, 1e-10);
    EXPECT_EQ(velocities.back().header.frame_id, "base_link");
    EXPECT_EQ(transforms.back().header.frame_id, "odom");
    EXPECT_EQ(transforms.back().child_frame_id, "base_link");
    EXPECT_DOUBLE_EQ(targets.back().pose.position.z, 40);
    for (int i = 25; i < 36; ++i) {frame(i, false);}
    const double last_magnetic_time = 1 + 24.0 / 30;
    EXPECT_LE(gv::seconds(positions.back().header.stamp), last_magnetic_time + 0.15 + 1e-6);
    EXPECT_LE(gv::seconds(targets.back().header.stamp), last_magnetic_time + 0.15 + 1e-6);
    for (const auto & node : {optical, navigation, guidance, driver}) {executor.remove_node(node);}
  }
}

TEST(TimeBuffer, IntegratesGyroAcrossUnevenKnots)
{
  gv::TimeBuffer<cv::Vec3d> buffer;
  buffer.push(0., {0., 0., 0.});
  buffer.push(1., {2., 0., 0.});
  buffer.push(3., {2., 0., 0.});
  const auto identity = [](const cv::Vec3d & v) {return v;};
  const auto mean = buffer.meanVector(.5, 2., 2., identity);
  ASSERT_TRUE(mean);
  EXPECT_NEAR((*mean)[0], 11. / 6., 1e-12);
  EXPECT_FALSE(buffer.meanVector(.5, 3.1, 2., identity));
  EXPECT_FALSE(buffer.meanVector(.5, 2., 1., identity));
}
