#include "spark_fast_lio.h"
#include <tf2_msgs/msg/tf_message.hpp>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <vector>

using Odom=nav_msgs::msg::Odometry;
static int64_t ns(const builtin_interfaces::msg::Time&t){return int64_t(t.sec)*1000000000+t.nanosec;}
int main(int argc,char**argv){
  if(!std::getenv("ROS_DOMAIN_ID") || std::string(std::getenv("ROS_DOMAIN_ID"))!="218")throw std::runtime_error("Synthetic probe requires isolated domain 218");
  rclcpp::init(argc,argv);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("common.visualization_frame","imu"),rclcpp::Parameter("common.base_frame",""),
    rclcpp::Parameter("common.map_frame","mid360_local_odom"),rclcpp::Parameter("common.imu_frame","mid360_imu")});
  auto node=std::make_shared<spark_fast_lio::SPARKFastLIO2>(options);
  auto observer=std::make_shared<rclcpp::Node>("phase_probe_observer");
  std::vector<Odom>corrected,predicted,mixed;std::vector<geometry_msgs::msg::TransformStamped>transforms;
  auto qos=rclcpp::QoS(100).reliable();
  auto a=observer->create_subscription<Odom>("odometry",qos,[&](Odom::ConstSharedPtr m){corrected.push_back(*m);});
  auto b=observer->create_subscription<Odom>("odometry_predicted",qos,[&](Odom::ConstSharedPtr m){predicted.push_back(*m);});
  auto c=observer->create_subscription<Odom>("odometry_mixed_legacy",qos,[&](Odom::ConstSharedPtr m){mixed.push_back(*m);});
  auto d=observer->create_subscription<tf2_msgs::msg::TFMessage>("/tf",rclcpp::QoS(100),[&](tf2_msgs::msg::TFMessage::ConstSharedPtr m){for(auto&t:m->transforms)if(t.header.frame_id=="mid360_local_odom"&&t.child_frame_id=="mid360_imu")transforms.push_back(t);});
  rclcpp::executors::SingleThreadedExecutor ex;ex.add_node(node);ex.add_node(observer);
  auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
  while((node->pub_odom_->get_subscription_count()!=1 || node->pub_odom_prediction_->get_subscription_count()!=1 || node->pub_odom_mixed_->get_subscription_count()!=1) && std::chrono::steady_clock::now()<deadline)ex.spin_some();
  if(node->pub_odom_->get_subscription_count()!=1)throw std::runtime_error("DDS endpoints missing");
  std::vector<int64_t>stamps={10005000000LL,10002000000LL,10010000000LL,10105000000LL,10102000000LL,10110000000LL};
  for(size_t i=0;i<stamps.size();i++){
    state_ikfom state;state.pos=Eigen::Vector3d(double(i),2.,3.);
    node->publishOdometry(state,rclcpp::Time(stamps[i]),i!=1&&i!=4);
    auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(30);
    while(std::chrono::steady_clock::now()<until)ex.spin_some();
  }
  deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(std::chrono::steady_clock::now()<deadline && (corrected.size()!=2||predicted.size()!=4||mixed.size()!=6||transforms.size()!=2))ex.spin_some();
  if(corrected.size()!=2||predicted.size()!=4||mixed.size()!=6||transforms.size()!=2)throw std::runtime_error("Output/TF count mismatch");
  for(size_t i=0;i<mixed.size();i++)if(ns(mixed[i].header.stamp)!=stamps[i]||mixed[i].pose.pose.position.x!=double(i))throw std::runtime_error("Original mixed sample lost or modified");
  for(size_t i=0;i<corrected.size();i++){
    size_t k=i?4:1;
    if(!(corrected[i]==mixed[k])||ns(transforms[i].header.stamp)!=stamps[k])throw std::runtime_error("Correction payload/TF mismatch");
  }
  std::vector<size_t>indices={0,2,3,5};
  for(size_t i=0;i<predicted.size();i++)if(!(predicted[i]==mixed[indices[i]]))throw std::runtime_error("Prediction payload mismatch");
  if(ns(corrected[0].header.stamp)>=ns(corrected[1].header.stamp))throw std::runtime_error("Corrected stamps regressed");
  for(size_t i=1;i<predicted.size();i++)if(ns(predicted[i-1].header.stamp)>=ns(predicted[i].header.stamp))throw std::runtime_error("Prediction stamps regressed");
  if(node->get_parameter("common.odometry_output_contract").as_string()!="scan_corrected_primary_v1")throw std::runtime_error("Wrong contract");
  std::cout<<"PASS actual production publishOdometry: mixed=6 (2 regressions preserved), corrected=2, predicted=4, TF=2 corrections only; timestamps/poses/payload unchanged; no filtering or retimestamping\n";
  ex.remove_node(node);ex.remove_node(observer);node.reset();observer.reset();rclcpp::shutdown();
}
