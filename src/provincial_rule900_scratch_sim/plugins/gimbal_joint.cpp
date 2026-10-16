// Real revolute yaw joint control and timestamped encoder measurement.
// No pose reset and no commanded angle presented as measured feedback.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <atomic>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Joint.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/double.pb.h>
#include <ignition/msgs/model.pb.h>
#include <ignition/msgs/boolean.pb.h>

namespace stage2 {
class GimbalJoint : public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure, public ignition::gazebo::ISystemPreUpdate,
  public ignition::gazebo::ISystemPostUpdate {
  ignition::gazebo::Joint joint{ignition::gazebo::kNullEntity};
  ignition::transport::Node node;
  ignition::transport::Node::Publisher publisher;
  std::mutex mutex;
  double target=0.;
  std::atomic<bool> encoderEnabled{true};
  std::chrono::steady_clock::duration previous{};
public:
  void Configure(const ignition::gazebo::Entity &entity,
      const std::shared_ptr<const sdf::Element> &, ignition::gazebo::EntityComponentManager &ecm,
      ignition::gazebo::EventManager &) override {
    ignition::gazebo::Model model(entity);
    joint=ignition::gazebo::Joint(model.JointByName(ecm,"gimbal_yaw"));
    if (!joint.Valid(ecm)) throw std::runtime_error("Missing physical gimbal_yaw joint");
    joint.EnablePositionCheck(ecm);joint.EnableVelocityCheck(ecm);
    publisher=node.Advertise<ignition::msgs::Model>("/gimbal/joint_states");
    node.Subscribe("/gimbal/target",&GimbalJoint::Target,this);
    node.Subscribe("/simulation/gimbal/feedback_enabled",&GimbalJoint::EnableFeedback,this);
  }
  void EnableFeedback(const ignition::msgs::Boolean &msg) {encoderEnabled=msg.data();}
  void Target(const ignition::msgs::Double &msg) {
    if (!std::isfinite(msg.data()) || std::abs(msg.data())>3.141592653589793) return;
    std::lock_guard<std::mutex> lock(mutex);target=msg.data();
  }
  void PreUpdate(const ignition::gazebo::UpdateInfo &info,
      ignition::gazebo::EntityComponentManager &ecm) override {
    if (info.paused) return;
    auto p=joint.Position(ecm);
    if (!p || p->empty()) return;
    std::lock_guard<std::mutex> lock(mutex);
    joint.SetVelocity(ecm,{std::clamp(4*(target-p->at(0)),-.4,.4)});
  }
  void PostUpdate(const ignition::gazebo::UpdateInfo &info,
      const ignition::gazebo::EntityComponentManager &ecm) override {
    if (info.paused || !encoderEnabled || info.simTime-previous<std::chrono::milliseconds(4)) return;
    auto p=joint.Position(ecm);auto v=joint.Velocity(ecm);
    if (!p || p->empty() || !v || v->empty()) return;
    previous=info.simTime;
    ignition::msgs::Model msg;msg.set_name("omni_robot");
    auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    msg.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);
    msg.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
    auto j=msg.add_joint();j->set_name("gimbal_yaw");
    j->mutable_axis1()->set_position(p->at(0));j->mutable_axis1()->set_velocity(v->at(0));
    publisher.Publish(msg);
  }
};
}
IGNITION_ADD_PLUGIN(stage2::GimbalJoint,ignition::gazebo::System,
  stage2::GimbalJoint::ISystemConfigure,stage2::GimbalJoint::ISystemPreUpdate,stage2::GimbalJoint::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(stage2::GimbalJoint,"stage2::GimbalJoint")
