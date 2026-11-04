#include "glider_visual_nav/ros_support.hpp"
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_air_data.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

namespace glider_visual_nav
{
class Px4Interface : public rclcpp::Node
{
public:
  Px4Interface() : Node("px4_interface_node")
  {
    regime_ = parameter(*this, "flight_regime", std::string("cruise_const_alt"));
    if (regime_ != "cruise_const_alt" && regime_ != "unpowered_glide") {throw std::invalid_argument("invalid flight_regime");}
    throttle_ = parameter(*this, "cruise_thrust", 0.55);
    cruise_speed_=parameter(*this,"cruise_speed",12.0);
    speed_kp_=parameter(*this,"thrust_speed_kp",.08);
    speed_ki_=parameter(*this,"thrust_speed_ki",.02);
    trim_ = parameter(*this, "bootstrap_pitch_up", 0.04);
    yaw_ = parameter(*this, "initial_yaw", 0.0);
    ground_baro_ = parameter(*this, "ground_baro_altitude", 0.0);
    timeout_ = parameter(*this, "offboard_visual_timeout", 0.5);
    auto_arm_ = parameter(*this, "auto_arm", false);
    for(double value : {throttle_,trim_,yaw_,ground_baro_,timeout_,cruise_speed_,speed_kp_,speed_ki_}) {
      if(!std::isfinite(value)) {throw std::invalid_argument("non-finite Offboard parameter");}
    }
    if (throttle_ < 0 || throttle_ > 1 || timeout_ <= 0 || cruise_speed_<=3 || speed_kp_<0 || speed_ki_<0 || std::abs(trim_)>.35)
    {throw std::invalid_argument("invalid Offboard limits");}
    const auto qos = rclcpp::SensorDataQoS();
    mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    setpoint_pub_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint_v1", 10);
    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("glider/imu", qos);
    height_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("glider/barometric_height", qos);
    state_pub_ = create_publisher<std_msgs::msg::String>("glider/interface_state", 10);
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", qos,
      [this](px4_msgs::msg::VehicleAttitude::ConstSharedPtr msg) {
        tf2::Quaternion q(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
        if (!std::isfinite(q.length2()) || std::abs(q.length2()-1) > .1) {return;}
        q.normalize(); const tf2::Matrix3x3 m(q);
        const cv::Matx33d nr(m[0][0],m[0][1],m[0][2],m[1][0],m[1][1],m[1][2],m[2][0],m[2][1],m[2][2]);
        const auto wb = enuToNed().t() * nr * fluToFrd();
        tf2::Matrix3x3(wb(0,0),wb(0,1),wb(0,2),wb(1,0),wb(1,1),wb(1,2),wb(2,0),wb(2,1),wb(2,2)).getRotation(attitude_);
        attitude_stamp_ = msg->timestamp_sample * 1e-6;
        attitudes_.push(attitude_stamp_, attitude_);
      });
    gyro_sub_ = create_subscription<px4_msgs::msg::VehicleAngularVelocity>("/fmu/out/vehicle_angular_velocity", qos,
      [this](px4_msgs::msg::VehicleAngularVelocity::ConstSharedPtr msg) {
        const double stamp = msg->timestamp_sample * 1e-6;
        const auto q = attitudes_.sample(stamp, .15, [](const auto & a, const auto & b, double t) {return a.slerp(b,t);});
        if (!q) {return;}
        sensor_msgs::msg::Imu imu;
        imu.header.frame_id = "base_link"; imu.header.stamp = rosStamp(stamp);
        imu.orientation.x=q->x(); imu.orientation.y=q->y(); imu.orientation.z=q->z(); imu.orientation.w=q->w();
        imu.angular_velocity.x=msg->xyz[0]; imu.angular_velocity.y=-msg->xyz[1]; imu.angular_velocity.z=-msg->xyz[2];
        imu.linear_acceleration_covariance[0]=-1; imu_pub_->publish(imu);
        imu_stamp_=stamp;
      });
    baro_sub_ = create_subscription<px4_msgs::msg::VehicleAirData>("/fmu/out/vehicle_air_data", qos,
      [this](px4_msgs::msg::VehicleAirData::ConstSharedPtr msg) {
        if (!std::isfinite(msg->baro_alt_meter)) {return;}
        geometry_msgs::msg::PointStamped height;
        height.header.stamp=rosStamp(msg->timestamp_sample*1e-6); height.header.frame_id="odom";
        height.point.z=msg->baro_alt_meter-ground_baro_; height_pub_->publish(height);
        height_stamp_=seconds(height.header.stamp);
      });
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status_v1", qos,
      [this](px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {status_=*msg; status_received_=now().seconds();});
    ack_sub_ = create_subscription<px4_msgs::msg::VehicleCommandAck>("/fmu/out/vehicle_command_ack", qos,
      [this](px4_msgs::msg::VehicleCommandAck::ConstSharedPtr msg) {
        RCLCPP_INFO(get_logger(), "PX4 command=%u result=%u", msg->command, msg->result);
      });
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("glider/target_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        if (msg->header.frame_id != "odom") {return;}
        tf2::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
        if (!std::isfinite(q.length2()) || std::abs(q.length2()-1) > .05) {return;}
        target_=*msg; target_stamp_=seconds(msg->header.stamp);
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("glider/odometry", 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto & v=msg->twist.twist.linear;
        if(msg->header.frame_id!="odom" || msg->child_frame_id!="base_link" || !validVector({v.x,v.y,v.z})) {return;}
        odom_stamp_=seconds(msg->header.stamp);
        speed_=std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z);
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {tick();});
  }
private:
  void command(uint32_t id, float a, float b=0, float c=0)
  {
    px4_msgs::msg::VehicleCommand msg; msg.timestamp=static_cast<uint64_t>(now().nanoseconds()/1000);
    msg.command=id; msg.param1=a; msg.param2=b; msg.param3=c;
    msg.target_system=1; msg.target_component=1; msg.source_system=42; msg.source_component=1;
    msg.from_external=true; command_pub_->publish(msg);
  }
  void transition(const std::string & state)
  {
    if (state != state_) {RCLCPP_INFO(get_logger(), "Offboard state: %s", state.c_str()); state_=state;}
    std_msgs::msg::String msg; msg.data=state_; state_pub_->publish(msg);
  }
  void tick()
  {
    const double time=now().seconds();
    const auto fresh=[&](double stamp, double limit) {return stamp >= 0 && time-stamp >= -.05 && time-stamp < limit;};
    if (!fresh(imu_stamp_, .5) || !fresh(height_stamp_, .5) || !fresh(status_received_, 2.0)) {
      transition("WAIT_SENSORS"); prestream_=-1; return; // Native PX4 Offboard-loss watchdog owns total sensor/process loss.
    }
    if (prestream_<0) {prestream_=time;}
    const bool armed=status_.arming_state==px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    const bool offboard=status_.nav_state==px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    const bool visual=fresh(odom_stamp_,timeout_);
    if (armed && offboard && !visual) {degraded_=true;}
    tf2::Quaternion desired;
    desired.setRPY(0,-trim_,yaw_);
    if (fresh(target_stamp_,timeout_) && visual && !degraded_) {
      const auto & q=target_.pose.orientation; desired=tf2::Quaternion(q.x,q.y,q.z,q.w);
    } else if (degraded_) {
      double r,p,y; tf2::Matrix3x3(attitude_).getRPY(r,p,y);
      desired.setRPY(0,-trim_,y); // wings-level best-glide fallback, no stale LOS or position hold
    }
    const tf2::Matrix3x3 m(desired);
    const auto nr=px4Attitude(cv::Matx33d(m[0][0],m[0][1],m[0][2],m[1][0],m[1][1],m[1][2],m[2][0],m[2][1],m[2][2]));
    tf2::Quaternion q;
    tf2::Matrix3x3(nr(0,0),nr(0,1),nr(0,2),nr(1,0),nr(1,1),nr(1,2),nr(2,0),nr(2,1),nr(2,2)).getRotation(q);
    px4_msgs::msg::OffboardControlMode mode; mode.timestamp=static_cast<uint64_t>(now().nanoseconds()/1000); mode.attitude=true;
    mode_pub_->publish(mode);
    px4_msgs::msg::VehicleAttitudeSetpoint setpoint; setpoint.timestamp=mode.timestamp;
    setpoint.q_d={static_cast<float>(q.w()),static_cast<float>(q.x()),static_cast<float>(q.y()),static_cast<float>(q.z())};
    double thrust=throttle_;
    if(degraded_) {thrust=.35;}
    else if(visual && armed && offboard && speed_>3) {
      const double error=cruise_speed_-speed_;
      const double dt=last_tick_>0 ? std::clamp(time-last_tick_,0.,.1) : 0;
      const double proposed=std::clamp(speed_integral_+error*dt,-20.,20.);
      const double unsaturated=throttle_+speed_kp_*error+speed_ki_*proposed;
      thrust=std::clamp(unsaturated,0.,1.);
      if(unsaturated==thrust || error*(unsaturated-thrust)<0) {speed_integral_=proposed;}
    }
    last_tick_=time;
    setpoint.thrust_body={regime_=="unpowered_glide" ? 0.0f : static_cast<float>(thrust),0.0f,0.0f};
    setpoint_pub_->publish(setpoint);
    if (time-prestream_<1.2) {transition("PRESTREAM"); return;}
    if (!auto_arm_) {transition("READY_MANUAL_ARM"); return;}
    if (time-last_command_>.8) {
      if (!offboard) {command(176,1,6);}
      else if (!armed) {command(400,1);}
      last_command_=time;
    }
    transition(!offboard ? "REQUEST_OFFBOARD" : !armed ? "REQUEST_ARM" : degraded_ ? "DEGRADED_WINGS_LEVEL" : "ACTIVE");
  }
  std::string regime_,state_; double throttle_,trim_,yaw_,ground_baro_,timeout_;
  double cruise_speed_,speed_kp_,speed_ki_,speed_{0},speed_integral_{0},last_tick_{-1};
  bool auto_arm_{false},degraded_{false};
  double attitude_stamp_{-1},imu_stamp_{-1},height_stamp_{-1},status_received_{-1},target_stamp_{-1},odom_stamp_{-1};
  double prestream_{-1},last_command_{-1}; tf2::Quaternion attitude_{0,0,0,1}; TimeBuffer<tf2::Quaternion> attitudes_;
  geometry_msgs::msg::PoseStamped target_; px4_msgs::msg::VehicleStatus status_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr height_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAngularVelocity>::SharedPtr gyro_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAirData>::SharedPtr baro_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};
}
int main(int argc,char ** argv)
{
  rclcpp::init(argc,argv); int result=0;
  try {rclcpp::spin(std::make_shared<glider_visual_nav::Px4Interface>());}
  catch(const std::exception & error) {RCLCPP_FATAL(rclcpp::get_logger("px4_interface_node"),"%s",error.what()); result=1;}
  rclcpp::shutdown(); return result;
}
