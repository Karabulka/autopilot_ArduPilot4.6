#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <chrono>
#include <iostream>
#include <optional>
#include <thread>
#include <unistd.h>

int main(int argc,char ** argv)
{
  if(argc<2) {return 2;}
  const bool publish=std::string(argv[1])=="pub";
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>(publish ? "dds_process_publisher" : "dds_process_subscriber");
  unsigned images=0,fields=0,sent=0; uint64_t checksum=0;
  bool warm_image=false,warm_field=false,ready=false;
  const auto qos=rclcpp::SensorDataQoS();
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr field_pub,ack_pub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr field_sub,ack_sub;
  if(publish) {
    image_pub=node->create_publisher<sensor_msgs::msg::Image>("dds_probe/image",qos);
    field_pub=node->create_publisher<sensor_msgs::msg::MagneticField>("dds_probe/field",qos);
    ack_sub=node->create_subscription<sensor_msgs::msg::MagneticField>("dds_probe/ready",qos,
      [&](sensor_msgs::msg::MagneticField::ConstSharedPtr msg) {ready=msg->magnetic_field.z==42e-6;});
  } else {
    ack_pub=node->create_publisher<sensor_msgs::msg::MagneticField>("dds_probe/ready",qos);
    image_sub=node->create_subscription<sensor_msgs::msg::Image>("dds_probe/image",qos,
      [&](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        if(msg->data.size()!=640U*480U || msg->data[123]!=123) {return;}
        if(msg->header.frame_id=="warmup") {warm_image=true;}
        else {++images; checksum+=msg->data[123];}
      });
    field_sub=node->create_subscription<sensor_msgs::msg::MagneticField>("dds_probe/field",qos,
      [&](sensor_msgs::msg::MagneticField::ConstSharedPtr msg) {
        if(msg->magnetic_field.y!=23e-6) {return;}
        if(msg->header.frame_id=="warmup") {warm_field=true;}
        else {++fields;}
      });
  }
  const auto start=std::chrono::steady_clock::now();
  auto last_received=start;
  std::optional<std::chrono::steady_clock::time_point> measured_start;
  unsigned tick=0;
  while(rclcpp::ok() && std::chrono::steady_clock::now()-start<std::chrono::seconds(23)) {
    const auto time=std::chrono::steady_clock::now();
    const auto before=fields;
    rclcpp::spin_some(node);
    if(fields!=before) {last_received=time;}
    if(publish) {
      // The remote process ACKs actual image AND mag delivery before measurement.
      // ROS discovery graph counts alone do not prove application readiness.
      if(ready && !measured_start) {measured_start=time;}
      if(measured_start && time-*measured_start>=std::chrono::seconds(4)) {break;}
      if(!ready && time-start>std::chrono::seconds(15)) {break;}
      sensor_msgs::msg::MagneticField field;
      field.header.frame_id=ready ? "base_link" : "warmup";
      field.header.stamp=node->now(); field.magnetic_field.y=23e-6; field_pub->publish(field);
      if(tick%3==0) {
        sensor_msgs::msg::Image image; image.header=field.header;
        image.width=640; image.height=480; image.step=640; image.encoding="mono8";
        image.data.resize(640*480); image.data[123]=123; image_pub->publish(image);
        if(ready) {++sent;}
      }
    } else {
      if(warm_image && warm_field && fields==0) {
        sensor_msgs::msg::MagneticField ack; ack.magnetic_field.z=42e-6; ack_pub->publish(ack);
      }
      if(fields>=200 && time-last_received>std::chrono::seconds(1)) {break;}
    }
    ++tick; std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::cout<<"{\"pid\":"<<getpid()<<",\"publisher\":"<<publish<<",\"ready\":"<<(publish ? ready : warm_image && warm_field)
    <<",\"images\":"<<images<<",\"fields\":"<<fields<<",\"sent_images\":"<<sent<<",\"checksum\":"<<checksum<<"}"<<std::endl;
  rclcpp::shutdown();
  return publish ? (ready ? 0 : 3) : (images>=70 && fields>=200 ? 0 : 4);
}
