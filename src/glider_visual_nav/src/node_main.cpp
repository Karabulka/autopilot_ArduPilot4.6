#include "glider_visual_nav/nodes.hpp"
#include <string>
#define GLIDER_STRING_IMPL(value) #value
#define GLIDER_STRING(value) GLIDER_STRING_IMPL(value)
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  const std::string name = GLIDER_STRING(GLIDER_NODE_NAME);
  try {
    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node;
    if (name == "optical_flow_node") {node = glider_visual_nav::makeOpticalFlowNode(options);}
    else if (name == "navigation_node") {node = glider_visual_nav::makeNavigationNode(options);}
    else {node = glider_visual_nav::makeLosControllerNode(options);}
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger(name), "%s", e.what()); status = 1;
  }
  rclcpp::shutdown(); return status;
}
