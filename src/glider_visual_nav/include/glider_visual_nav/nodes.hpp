#pragma once
#include <rclcpp/rclcpp.hpp>
namespace glider_visual_nav
{
rclcpp::Node::SharedPtr makeOpticalFlowNode(const rclcpp::NodeOptions & options);
rclcpp::Node::SharedPtr makeNavigationNode(const rclcpp::NodeOptions & options);
rclcpp::Node::SharedPtr makeLosControllerNode(const rclcpp::NodeOptions & options);
}  // namespace glider_visual_nav
