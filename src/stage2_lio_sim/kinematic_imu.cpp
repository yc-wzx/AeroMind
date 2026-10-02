// VelocityControl changes velocity without a useful IMU acceleration component
// in this Fortress setup. Measure velocity of the physical sensor point instead.
// This sensor system NEVER sets forces, velocities, pose, or navigation inputs.
#include <chrono>
#include <memory>
#include <sstream>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/sensors/ImuSensor.hh>
#include <ignition/sensors/SensorFactory.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/stringmsg.pb.h>
#include <sdf/Sensor.hh>

namespace stage2 {
class KinematicImu : public ignition::gazebo::System,
 public ignition::gazebo::ISystemConfigure, public ignition::gazebo::ISystemPostUpdate {
 private: ignition::gazebo::Link link{ignition::gazebo::kNullEntity};
 private: std::unique_ptr<ignition::sensors::ImuSensor> sensor;
 private: ignition::math::Vector3d offset{0,0,.25}, previousVelocity;
 private: std::chrono::steady_clock::duration previousTime{};
 private: bool initialized=false;
 private: ignition::transport::Node node;
 private: ignition::transport::Node::Publisher diagnostics;
 public: void Configure(const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm, ignition::gazebo::EventManager &) override {
   ignition::gazebo::Model model(_entity);
   link=ignition::gazebo::Link(model.LinkByName(_ecm,"base_link"));
   if (!link.Valid(_ecm)) throw std::runtime_error("IMU requires base_link");
   link.EnableVelocityChecks(_ecm);
   auto element=const_cast<sdf::Element *>(_sdf.get())->GetElement("sensor");
   sdf::Sensor definition;auto errors=definition.Load(element);
   if (!errors.empty()) throw std::runtime_error("Invalid kinematic IMU SDF");
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
   auto velocity=link.WorldLinearVelocity(_ecm,offset);
   auto angular=link.WorldAngularVelocity(_ecm);
   auto pose=link.WorldPose(_ecm);
   // Missing data never become zero measurements. Reinitialize a real prefix.
   if (!velocity || !angular || !pose) { initialized=false; return; }
   if (!initialized) {
     previousVelocity=*velocity;previousTime=_info.simTime;initialized=true;return;
   }
   const double dt=std::chrono::duration<double>(_info.simTime-previousTime).count();
   if (dt<=0 || dt>.02) {
     previousVelocity=*velocity;previousTime=_info.simTime;return;
   }
   auto accel=(*velocity-previousVelocity)/dt;
   previousVelocity=*velocity;previousTime=_info.simTime;
   const auto sensorPose=*pose * ignition::math::Pose3d(offset,ignition::math::Quaterniond::Identity);
   const auto localAccel=sensorPose.Rot().Inverse().RotateVector(accel);
   const auto localAngular=sensorPose.Rot().Inverse().RotateVector(*angular);
   sensor->SetWorldPose(sensorPose);sensor->SetLinearAcceleration(localAccel);
   sensor->SetAngularVelocity(localAngular);
   static_cast<ignition::sensors::Sensor *>(sensor.get())->Update(_info.simTime,false);
   std::ostringstream out;out.precision(17);
   out << "{\"stamp_s\":" << std::chrono::duration<double>(_info.simTime).count()
       << ",\"dt_s\":" << dt << ",\"world_velocity\":[" << velocity->X() << ',' << velocity->Y() << ',' << velocity->Z()
       << "],\"world_acceleration\":[" << accel.X() << ',' << accel.Y() << ',' << accel.Z()
       << "],\"body_acceleration\":[" << localAccel.X() << ',' << localAccel.Y() << ',' << localAccel.Z()
       << "],\"body_angular_velocity\":[" << localAngular.X() << ',' << localAngular.Y() << ',' << localAngular.Z()
       << "],\"source\":\"physics sensor-point velocity; no ROS GT input\"}";
   ignition::msgs::StringMsg m;m.set_data(out.str());diagnostics.Publish(m);
 }
};
}
IGNITION_ADD_PLUGIN(stage2::KinematicImu,ignition::gazebo::System,
 stage2::KinematicImu::ISystemConfigure,stage2::KinematicImu::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(stage2::KinematicImu,"stage2::KinematicImu")
