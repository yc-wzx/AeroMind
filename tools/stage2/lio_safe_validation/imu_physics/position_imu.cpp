// Opt-in simulated sensor. Velocity caches can retain commanded motion after
// VelocityControl stops. Differentiate actual sensor-point world positions.
// No ROS GT subscriber, commanded velocity input, force or pose setter.
#include <chrono>
#include <memory>
#include <sstream>
#include <cmath>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/sensors/ImuSensor.hh>
#include <ignition/sensors/SensorFactory.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/stringmsg.pb.h>
#include <sdf/Sensor.hh>

namespace stage2 {
class PositionImu : public ignition::gazebo::System,
 public ignition::gazebo::ISystemConfigure, public ignition::gazebo::ISystemPostUpdate {
 private: ignition::gazebo::Link link{ignition::gazebo::kNullEntity};
 private: std::unique_ptr<ignition::sensors::ImuSensor> sensor;
 private: ignition::math::Vector3d offset{0,0,.25}, previousPosition, previousVelocity;
 private: ignition::math::Quaterniond previousRotation;
 private: std::chrono::steady_clock::duration previousTime{};
 private: unsigned int prefix=0;
 private: ignition::transport::Node node;
 private: ignition::transport::Node::Publisher diagnostics;
 public: void Configure(const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm, ignition::gazebo::EventManager &) override {
   ignition::gazebo::Model model(_entity);
   link=ignition::gazebo::Link(model.LinkByName(_ecm,"base_link"));
   if (!link.Valid(_ecm)) throw std::runtime_error("IMU requires base_link");
   auto element=const_cast<sdf::Element *>(_sdf.get())->GetElement("sensor");
   sdf::Sensor definition;auto errors=definition.Load(element);
   if (!errors.empty()) throw std::runtime_error("Invalid position IMU SDF");
   definition.SetName("omni_robot/base_link/imu3d");
   offset=definition.RawPose().Pos();
   ignition::sensors::SensorFactory factory;
   sensor=factory.CreateSensor<ignition::sensors::ImuSensor>(definition);
   if (!sensor) throw std::runtime_error("Could not instantiate IMU sensor");
   sensor->SetGravity({0,0,-9.81});sensor->SetOrientationEnabled(false);
   diagnostics=node.Advertise<ignition::msgs::StringMsg>("/simulation/imu_kinematics");
 }
 public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm) override {
   if (_info.paused || !sensor) return;
   // Recompute through current Pose components, not a cached WorldPose or
   // WorldLinearVelocity component. Same physical scene used by rendering.
   const auto pose=ignition::gazebo::worldPose(link.Entity(),_ecm) *
     ignition::math::Pose3d(offset,ignition::math::Quaterniond::Identity);
   const auto position=pose.Pos();
   if (prefix==0) {
     previousPosition=position;previousRotation=pose.Rot();
     previousTime=_info.simTime;prefix=1;return;
   }
   const double dt=std::chrono::duration<double>(_info.simTime-previousTime).count();
   if (dt<=0 || dt>.02) {prefix=0;return;}
   const auto velocity=(position-previousPosition)/dt;
   auto dq=previousRotation.Inverse()*pose.Rot();
   if (dq.W()<0) dq=ignition::math::Quaterniond(-dq.W(),-dq.X(),-dq.Y(),-dq.Z());
   const double norm=std::sqrt(dq.X()*dq.X()+dq.Y()*dq.Y()+dq.Z()*dq.Z());
   const double scale=norm>1e-12 ? 2*std::atan2(norm,dq.W())/(norm*dt) : 2/dt;
   const auto worldAngular=previousRotation.RotateVector(
      ignition::math::Vector3d(dq.X()*scale,dq.Y()*scale,dq.Z()*scale));
   const auto acceleration=(velocity-previousVelocity)/dt;
   previousPosition=position;previousRotation=pose.Rot();previousTime=_info.simTime;
   if (prefix==1) {previousVelocity=velocity;prefix=2;return;}
   previousVelocity=velocity;
   const auto localAccel=pose.Rot().Inverse().RotateVector(acceleration);
   const auto localAngular=pose.Rot().Inverse().RotateVector(worldAngular);
   sensor->SetWorldPose(pose);sensor->SetLinearAcceleration(localAccel);
   sensor->SetAngularVelocity(localAngular);
   static_cast<ignition::sensors::Sensor *>(sensor.get())->Update(_info.simTime,false);
   std::ostringstream out;out.precision(17);
   out << "{\"stamp_s\":" << std::chrono::duration<double>(_info.simTime).count()
       << ",\"dt_s\":" << dt
       << ",\"world_position\":[" << position.X() << ',' << position.Y() << ',' << position.Z()
       << "],\"world_velocity\":[" << velocity.X() << ',' << velocity.Y() << ',' << velocity.Z()
       << "],\"world_acceleration\":[" << acceleration.X() << ',' << acceleration.Y() << ',' << acceleration.Z()
       << "],\"body_acceleration\":[" << localAccel.X() << ',' << localAccel.Y() << ',' << localAccel.Z()
       << "],\"body_angular_velocity\":[" << localAngular.X() << ',' << localAngular.Y() << ',' << localAngular.Z()
       << "],\"source\":\"actual physical sensor-point positions; interval-end finite differences; no ROS GT or command input\"}";
   ignition::msgs::StringMsg message;message.set_data(out.str());diagnostics.Publish(message);
 }
};
}
IGNITION_ADD_PLUGIN(stage2::PositionImu,ignition::gazebo::System,
 stage2::PositionImu::ISystemConfigure,stage2::PositionImu::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(stage2::PositionImu,"stage2::PositionImu")
