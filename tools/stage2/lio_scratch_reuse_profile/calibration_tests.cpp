#include "spark_fast_lio.h"
#include <iostream>
namespace aeromind_input {void process_fixture_bridge(ImuProcess&,const MeasureGroup&,esekfom::esekf<state_ikfom,12,input_ikfom>&,PointCloudXYZI::Ptr);}
struct PredictionProbe {
 static int count,fail;static void ck(const char*n,bool b){count++;if(!b)fail++;std::cout<<n<<" "<<b<<"\n";}
 static auto node(){rclcpp::NodeOptions o;o.parameter_overrides({rclcpp::Parameter("mapping.acc_cov",1e-6),rclcpp::Parameter("mapping.gyr_cov",1e-8),rclcpp::Parameter("common.base_frame",std::string("")),rclcpp::Parameter("common.planar_mode",false),rclcpp::Parameter("common.visualization_frame",std::string("imu")),rclcpp::Parameter("gravity_alignment.enable_gravity_alignment",false)});return std::make_shared<spark_fast_lio::SPARKFastLIO2>(o);}
 static MeasureGroup group(double begin,double end,int first,int last,bool tilted=false){MeasureGroup m;m.lidar_beg_time=begin;m.lidar_end_time=end;m.lidar=std::make_shared<PointCloudXYZI>();for(int j=0;j<5;j++){PointType p{};p.x=2+j*.05;p.y=.1;p.z=.2;p.curvature=0;m.lidar->push_back(p);}for(int i=first;i<=last;i++){auto x=std::make_shared<sensor_msgs::msg::Imu>();x->header.stamp=rclcpp::Time(int64_t(std::llround((10+i*.004)*1e9)));x->angular_velocity.x=(i%2?1:-1)*.0001;x->linear_acceleration.x=tilted?.5:0;x->linear_acceleration.z=tilted?std::sqrt(9.81*9.81-.25):9.81;m.imu.push_back(x);}return m;}
 static void call(spark_fast_lio::SPARKFastLIO2&n,const MeasureGroup&m,PointCloudXYZI::Ptr o){aeromind_input::process_fixture_bridge(*n.imu_processor_,m,n.kf_,o);}
 static void run(){auto n=node();auto out=std::make_shared<PointCloudXYZI>();call(*n,group(10,11.2,1,300),out);ck("short_window_stays_initializing",n->imu_processor_->imu_need_init_&&!n->imu_processor_->propagation_anchor_valid_&&out->empty());call(*n,group(11.2,14.8,301,1200),out);ck("4_8_seconds_not_ready",n->imu_processor_->imu_need_init_);call(*n,group(14.8,15,1201,1250),out);ck("5_seconds_and1250_actual_samples_ready",!n->imu_processor_->imu_need_init_&&n->imu_processor_->propagation_anchor_valid_&&n->imu_processor_->init_iter_num==1251);
 ck("gravity_matches_normalization_constant",std::abs(n->kf_.get_x().grav.length-9.81)<1e-14&&std::abs(V3D(n->kf_.get_x().grav).norm()-9.81)<1e-14);
 ck("stationary_acceleration_cache_no_artificial_001_residual",n->imu_processor_->acc_s_last.norm()<1e-12);
 ck("gyro_prior_exact_sample_mean_variance",std::abs(n->kf_.get_P()(15,15)-.0001*.0001/1249)<1e-20);
 ck("accel_prior_declared_noise_sample_mean",std::abs(n->kf_.get_P()(18,18)-1e-6/1249)<1e-20);
 ck("gravity_direction_prior_from_mean_uncertainty",std::abs(n->kf_.get_P()(21,21)-2e-6/1249/(9.81*9.81))<1e-22);
 ck("accelerometer_mean_not_forced",n->kf_.get_x().ba.norm()==0 && std::abs(n->imu_processor_->mean_acc[2]-9.81)<1e-12);
 ck("positive_finite_covariance",n->kf_.get_P().allFinite()&&n->kf_.get_P().diagonal().minCoeff()>0);
 ck("velocity_prior_not_zeroed",n->kf_.get_P()(12,12)==1.);
 auto n2=node();auto x=n2->kf_.get_x();x.vel<<.1,.2,.3;n2->kf_.change_x(x);call(*n2,group(10,15,1,1250,true),out);
 ck("tilted_gravity_not_forced_vertical",std::abs(n2->kf_.get_x().grav[0])>.49&&std::abs(V3D(n2->kf_.get_x().grav).norm()-9.81)<1e-12);
 ck("initialization_does_not_clamp_velocity",(n2->kf_.get_x().vel-Eigen::Vector3d(.1,.2,.3)).norm()==0);
 ck("original_stationary_assumption_retained",!n2->imu_processor_->imu_need_init_);
 }
};int PredictionProbe::count=0,PredictionProbe::fail=0;
int main(int argc,char**argv){rclcpp::init(argc,argv);PredictionProbe::run();rclcpp::shutdown();std::cout<<"checks="<<PredictionProbe::count<<" failures="<<PredictionProbe::fail<<"\n";return PredictionProbe::fail?2:0;}
