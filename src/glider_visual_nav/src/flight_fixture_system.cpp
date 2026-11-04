// Test fixture + evaluator-only truth. Never loaded as a navigation measurement.
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/Utility.hh>
#include <mutex>
#include <atomic>
#include <limits>

namespace glider_visual_nav
{

class FlightFixtureSystem : public gz::sim::System, public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate
{
public:
  void Configure(const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {

    world_=entity;
    if (sdf && sdf->HasElement("model_name")) {model_name_=sdf->Get<std::string>("model_name");}
    publisher_=node_.Advertise<gz::msgs::Odometry>("/evaluation/ground_truth");
    motor_publisher_=node_.Advertise<gz::msgs::Double>("/evaluation/motor_speed_cmd");
    node_.Subscribe("/rc_cessna_visual/command/motor_speed", &FlightFixtureSystem::motor, this);
    node_.Advertise("/evaluation/launch", &FlightFixtureSystem::launch, this);
  }
  void motor(const gz::msgs::Actuators & msg)
  {if(msg.velocity_size()>0) {motor_command_.store(msg.velocity(0));}}
  bool launch(const gz::msgs::Vector3d & request, gz::msgs::Boolean & reply)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const gz::math::Vector3d v(request.x(),request.y(),request.z());
    const bool valid=!released_ && link_!=gz::sim::kNullEntity && v.IsFinite() && v.Length()>=8 && v.Length()<=30;
    if(valid) {velocity_=v; pending_=true;}
    reply.set_data(valid); return true;
  }
  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override
  {
    if(info.paused) {return;}
    std::lock_guard<std::mutex> lock(mutex_);
    if(link_==gz::sim::kNullEntity) {
      const auto model=gz::sim::World(world_).ModelByName(ecm,model_name_);
      const auto anchor=gz::sim::World(world_).ModelByName(ecm,"launch_anchor");
      if(model==gz::sim::kNullEntity || anchor==gz::sim::kNullEntity) {return;}
      model_=model;
      link_=gz::sim::Model(model).LinkByName(ecm,"base_link");
      const auto parent=gz::sim::Model(anchor).LinkByName(ecm,"anchor");
      if(link_==gz::sim::kNullEntity || parent==gz::sim::kNullEntity) {link_=gz::sim::kNullEntity; return;}
      gz::sim::Link(link_).EnableVelocityChecks(ecm);
      joint_=ecm.CreateEntity();
      ecm.CreateComponent(joint_,gz::sim::components::DetachableJoint({parent,link_,"fixed"}));
    }
    if(pending_ && !released_) {
      ecm.RequestRemoveEntity(joint_);
      for(auto entity:gz::sim::Model(model_).Links(ecm)) {
        const auto pose=gz::sim::worldPose(entity,ecm);
        gz::sim::Link(entity).SetLinearVelocity(ecm,pose.Rot().RotateVectorReverse(velocity_));
        gz::sim::Link(entity).SetAngularVelocity(ecm,gz::math::Vector3d::Zero);
      }
      pending_=false; released_=true;
    }
  }
  void PostUpdate(const gz::sim::UpdateInfo & info,const gz::sim::EntityComponentManager & ecm) override
  {
    if(info.paused || link_==gz::sim::kNullEntity || info.simTime-last_publish_<std::chrono::milliseconds(16)) {return;}
    const auto pose=gz::sim::worldPose(link_,ecm);
    const auto velocity=gz::sim::Link(link_).WorldLinearVelocity(ecm);
    const auto omega=gz::sim::Link(link_).WorldAngularVelocity(ecm);
    if(!velocity || !omega) {return;}
    gz::msgs::Odometry msg;
    const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    msg.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);
    msg.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
    auto frame=msg.mutable_header()->add_data(); frame->set_key("frame_id"); frame->add_value("odom");
    auto child=msg.mutable_header()->add_data(); child->set_key("child_frame_id"); child->add_value("base_link");
    gz::msgs::Set(msg.mutable_pose(),pose);
    gz::msgs::Set(msg.mutable_twist()->mutable_linear(),pose.Rot().RotateVectorReverse(*velocity));
    gz::msgs::Set(msg.mutable_twist()->mutable_angular(),pose.Rot().RotateVectorReverse(*omega));
    publisher_.Publish(msg); last_publish_=info.simTime;
    if(std::isfinite(motor_command_.load())) {
      gz::msgs::Double motor_msg; motor_msg.set_data(motor_command_.load()); motor_publisher_.Publish(motor_msg);
    }
  }
private:
  std::string model_name_{"rc_cessna_visual"};
  gz::sim::Entity world_{gz::sim::kNullEntity},model_{gz::sim::kNullEntity},link_{gz::sim::kNullEntity},joint_{gz::sim::kNullEntity};
  bool pending_{false},released_{false}; gz::math::Vector3d velocity_;
  std::mutex mutex_; gz::transport::Node node_; gz::transport::Node::Publisher publisher_;
  gz::transport::Node::Publisher motor_publisher_;
  std::atomic<double> motor_command_{std::numeric_limits<double>::quiet_NaN()};
  std::chrono::steady_clock::duration last_publish_{};
};
}
GZ_ADD_PLUGIN(glider_visual_nav::FlightFixtureSystem,gz::sim::System,gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate,gz::sim::ISystemPostUpdate)
