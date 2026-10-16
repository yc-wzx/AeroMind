#include "spark_fast_lio.h"
#include "input_bundle_trace.hpp"
#include <rclcpp/serialization.hpp>
#include <jsoncpp/json/json.h>
#include <fstream>
#include <iostream>
#include <cstring>
#include <cmath>
struct PredictionProbe {
 using Node=spark_fast_lio::SPARKFastLIO2;
 static void replay(Node &n,const Json::Value &actions,std::ifstream &payload,std::ostream &out){
 n.main_loop_timer_->cancel();
 for(const auto&a:actions){std::string k=a["kind"].asString();
 if(k=="IMU_CALLBACK_ENTRY"){
  auto s=a["sample"];if(s["values"].size()!=37)throw std::runtime_error("IMU field count");auto m=std::make_shared<sensor_msgs::msg::Imu>();
  const auto t=s["stamp_ns"].asInt64();m->header.stamp.sec=t/1000000000;m->header.stamp.nanosec=t%1000000000;m->header.frame_id=s["frame"].asString();
  double v[37];for(int i=0;i<37;i++){v[i]=s["values"][i].asDouble();if(!s["values"][i].isNumeric()||!std::isfinite(v[i]))throw std::runtime_error("nonfinite IMU");}
  m->orientation.x=v[0];m->orientation.y=v[1];m->orientation.z=v[2];m->orientation.w=v[3];m->angular_velocity.x=v[4];m->angular_velocity.y=v[5];m->angular_velocity.z=v[6];m->linear_acceleration.x=v[7];m->linear_acceleration.y=v[8];m->linear_acceleration.z=v[9];
  for(int i=0;i<9;i++){m->orientation_covariance[i]=v[10+i];m->angular_velocity_covariance[i]=v[19+i];m->linear_acceleration_covariance[i]=v[28+i];}n.imuCallback(m);
 }else if(k=="LIDAR_CALLBACK_ENTRY"){
  size_t sz=a["bytes"].asUInt64();if(!sz||sz>4000000)throw std::runtime_error("CDR size");rclcpp::SerializedMessage wire(sz);auto &buf=wire.get_rcl_serialized_message();payload.seekg(a["offset"].asUInt64());payload.read((char*)buf.buffer,sz);if(size_t(payload.gcount())!=sz)throw std::runtime_error("CDR truncated");buf.buffer_length=sz;
  sensor_msgs::msg::PointCloud2 m;rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;serializer.deserialize_message(&wire,&m);
  if(int64_t(m.header.stamp.sec)*1000000000LL+m.header.stamp.nanosec!=a["stamp_ns"].asInt64())throw std::runtime_error("lidar identity mismatch");n.standardLiDARCallback(m);
 }else if(k=="SYNC_BUNDLE"){
  n.main();aeromind_input::Event e;e.kind="DIRECT_MAIN_POST";e.lidar_begin=a["lidar_begin"].asDouble();e.lidar_end=a["lidar_end"].asDouble();e.latest_received_imu_ns=n.last_imu_timestamp_.nanoseconds();aeromind_input::filter(e,n.kf_);e.entry_ns=e.capture_done_ns=1;uint64_t invalid=0;out<<aeromind_input::serialize(e,invalid);if(invalid)throw std::runtime_error("nonfinite post");
 }else throw std::runtime_error("unknown action");
 }
 }
};
int main(int argc,char**argv){try{if(argc<5)throw std::runtime_error("schedule payload output ROS args");std::ifstream f(argv[1]);Json::Value root;f>>root;if(!root.isArray()||root.size()<100)throw std::runtime_error("schedule incomplete");std::ifstream payload(argv[2],std::ios::binary);std::ofstream out(argv[3]);if(!payload||!out)throw std::runtime_error("file open");rclcpp::init(argc-3,argv+3);rclcpp::NodeOptions o;auto n=std::make_shared<spark_fast_lio::SPARKFastLIO2>(o);PredictionProbe::replay(*n,root,payload,out);n.reset();rclcpp::shutdown();std::cout<<"fixture complete "<<root.size()<<std::endl;return 0;}catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 2;}}
