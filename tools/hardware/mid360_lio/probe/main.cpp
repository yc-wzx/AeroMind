#include "spark_fast_lio.h"
#include <iostream>
#include <stdexcept>

int main(int argc,char**argv) {
  rclcpp::init(argc,argv);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("common.visualization_frame","imu"),
                              rclcpp::Parameter("common.map_frame","mid360_local_odom"),
                              rclcpp::Parameter("common.imu_frame","mid360_imu"),
                              rclcpp::Parameter("common.base_frame","")});
  auto node=std::make_shared<spark_fast_lio::SPARKFastLIO2>(options);
  state_ikfom state;
  state.pos=Eigen::Vector3d(1.,2.,3.);
  geometry_msgs::msg::PoseWithCovariance pose;
  bool rejected=false;
  try {node->setPoseStamp(state,pose,"mid360_imu");}
  catch(const std::invalid_argument&) {rejected=true;}
  if(!rejected)throw std::runtime_error("Old bad mode counterexample did not reject");
  node->setPoseStamp(state,pose,"imu");
  if(pose.pose.position.x!=1. || pose.pose.position.y!=2. || pose.pose.position.z!=3.)
    throw std::runtime_error("Correct mode lost sensor pose");
  // Actual production output method exercises its second mode dispatch as well.
  node->publishOdometry(state,rclcpp::Time(10000000000LL));
  if(node->odomAftMapped_.header.frame_id!="mid360_local_odom" || node->odomAftMapped_.child_frame_id!="mid360_imu")
    throw std::runtime_error("Actual production output frame mismatch");
  std::cout<<"PASS: old visualization mode mid360_imu throws; corrected imu mode and actual publishOdometry preserve pose / correct frames; synthetic isolated test only\n";
  node.reset();rclcpp::shutdown();
}
