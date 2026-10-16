#include "spark_fast_lio.h"
#include <iostream>
#include <iomanip>
#include <limits>
namespace aeromind_input {void process_fixture_bridge(ImuProcess&,const MeasureGroup&,esekfom::esekf<state_ikfom,12,input_ikfom>&,PointCloudXYZI::Ptr);}
struct PredictionProbe {
 using Node=spark_fast_lio::SPARKFastLIO2;static int checks,failures;
 static void check(const char*n,bool v){++checks;if(!v)++failures;std::cout<<"{\"test\":\""<<n<<"\",\"pass\":"<<(v?"true":"false")<<"}\n";}
 static auto imu(double t,double ax=0.){auto m=std::make_shared<sensor_msgs::msg::Imu>();m->header.stamp=rclcpp::Time(int64_t(std::llround(t*1e9)));m->linear_acceleration.x=ax;m->linear_acceleration.z=G_m_s2;m->angular_velocity.x=.00001;m->angular_velocity.y=.00002;return m;}
 static MeasureGroup group(double begin,double end,const std::vector<double>&times,double ax=0.){MeasureGroup m;m.lidar_beg_time=begin;m.lidar_end_time=end;m.lidar=std::make_shared<PointCloudXYZI>();for(int i=0;i<5;i++){PointType p{};p.x=2.f+i*.05f;p.y=.1f;p.z=.2f;p.curvature=0;m.lidar->push_back(p);}for(double t:times)m.imu.push_back(imu(t,ax));return m;}
 static auto node(){rclcpp::NodeOptions o;o.parameter_overrides({rclcpp::Parameter("gravity_alignment.enable_gravity_alignment",false),rclcpp::Parameter("pcl_verbose",false),rclcpp::Parameter("common.base_frame",std::string("")),rclcpp::Parameter("common.visualization_frame",std::string("imu"))});return std::make_shared<Node>(o);}
 static void call(Node&n,const MeasureGroup&m,PointCloudXYZI::Ptr out){aeromind_input::process_fixture_bridge(*n.imu_processor_,m,n.kf_,out);}
 static auto boot(Node&n,double base=10.,double end_offset=.041){std::vector<double>times;for(int i=1;i<=10;i++)times.push_back(base+i*.004);auto m=group(base,base+end_offset,times);auto out=std::make_shared<PointCloudXYZI>();call(n,m,out);return m;}
 static void run(){
 if(std::getenv("AEROMIND_EMPTY_FIXTURE")){auto n=node();auto out=std::make_shared<PointCloudXYZI>();auto init=group(50.,50.012,{50.004,50.008,50.012});call(*n,init,out);auto before=n->kf_.get_x();auto P=n->kf_.get_P();auto empty=group(50.014,50.020,{});out->push_back(PointType{});auto size=out->size();call(*n,empty,out);check("empty_actual_production_preserves_full_covariance",(n->kf_.get_P()-P).norm()==0.);check("empty_preserves_state_and_output",(n->kf_.get_x().pos-before.pos).norm()==0.&&out->size()==size);check("empty_preserves_init_count",n->imu_processor_->init_iter_num==4);return;}
 auto n=node();auto &p=*n->imu_processor_;auto out=std::make_shared<PointCloudXYZI>();check("constructor_invalid_anchor_and_finite_cache",!p.propagation_anchor_valid_&&p.last_lidar_end_time_==-1.&&p.acc_s_last.isZero()&&p.acc_s_last.allFinite());
 auto initial=boot(*n);check("actual_init_complete",!p.imu_need_init_&&p.init_iter_num==11&&p.propagation_anchor_valid_);check("anchor_is_init_scan_end",p.last_lidar_end_time_==10.041);check("anchor_not_epoch_zero_or_tail_time",p.last_lidar_end_time_!=0.&&p.last_lidar_end_time_!=rclcpp::Time(p.last_imu_->header.stamp).seconds());double vertical_residual=G_m_s2+V3D(n->kf_.get_x().grav).z();std::cout<<std::setprecision(17)<<"{\"retained_vertical_acceleration_residual\":"<<vertical_residual<<"}\n";check("cache_matches_existing_gravity_convention",std::abs(p.acc_s_last.z()-vertical_residual)<1e-12&&std::abs(p.acc_s_last.x())<1e-12&&std::abs(p.acc_s_last.y())<1e-12);check("cached_gyro_bias_corrected",p.angvel_last.norm()<1e-12);
 auto initialCov=n->kf_.get_P();
 for(int j=0;j<6;j++)check("local_frame_anchor_numerical_regularizer",std::abs(initialCov(j,j)-1e-9)<1e-18);
 for(int j=0;j<3;j++)check("gyro_bias_variance_from_startup_sample_mean",std::abs(initialCov(15+j,15+j)-1e-12)<1e-18);
 check("velocity_prior_not_clamped",initialCov(12,12)==1&&initialCov(13,13)==1&&initialCov(14,14)==1);
 check("gravity_and_accelerometer_bias_priors_retained",initialCov(18,18)==.001&&initialCov(21,21)==.00001);
 auto before=n->kf_.get_x();auto cov=n->kf_.get_P();auto moving=group(10.05,10.053,{10.044,10.048,10.052},.2);call(*n,moving,out);auto actual=n->kf_.get_x();
 // Independent Euler-position recurrence with the production pair averages and clipped durations.
 // .003*.1, .004*.2, .004*.2 and final .001*.2; initialization interval is not integrated again.
 check("first_propagation_complete_clipped_velocity",std::abs(actual.vel.x()-.0021)<1e-12);check("first_propagation_complete_clipped_position",std::abs(actual.pos.x()-.0000075)<1e-12);check("cross_axis_motion_matches_existing_gravity",std::abs(actual.pos.y())<1e-12&&std::abs(actual.pos.z()-vertical_residual*.000051)<1e-12);check("first_output_and_new_anchor",out->size()==5&&p.last_lidar_end_time_==10.053);
 auto once=n->kf_.get_x();auto onceP=n->kf_.get_P();call(*n,moving,out);check("duplicate_scan_no_output",out->empty());check("duplicate_scan_no_KF_change",(n->kf_.get_x().pos-once.pos).norm()==0.&&(n->kf_.get_P()-onceP).norm()==0.);
 auto stale=group(10.054,10.060,{10.052});call(*n,stale,out);check("no_forward_IMU_no_output_or_state_change",out->empty()&&(n->kf_.get_x().pos-once.pos).norm()==0.&&p.last_lidar_end_time_==10.053);
 auto bad=group(10.06,std::numeric_limits<double>::quiet_NaN(),{10.056});call(*n,bad,out);check("NaN_scan_interval_rejected",out->empty()&&p.last_lidar_end_time_==10.053);
 bad=group(10.06,10.05,{10.044});call(*n,bad,out);check("reversed_scan_interval_rejected",out->empty()&&(n->kf_.get_P()-onceP).norm()==0.);
 bad=group(10.06,10.061,{10.064});call(*n,bad,out);check("future_tail_outside_scan_rejected",out->empty()&&p.last_lidar_end_time_==10.053);
 p.Reset();check("reset_invalidates_old_anchor",!p.propagation_anchor_valid_&&p.last_lidar_end_time_==-1.);check("reset_clears_motion_caches",p.acc_s_last.isZero()&&p.angvel_last.isZero());check("reset_restarts_first_init_flag",p.b_first_frame_&&p.imu_need_init_&&p.init_iter_num==1);auto reboot=boot(*n,20.);check("new_epoch_anchor_not_old_anchor",p.propagation_anchor_valid_&&p.last_lidar_end_time_==20.041&&p.first_lidar_time==20.);
 // State itself is intentionally caller-owned across Reset; verify finite controlled restarted propagation.
 auto next=group(20.042,20.052,{20.044,20.048,20.052});call(*n,next,out);check("post_reset_forward_propagation_finite",out->size()==5&&n->kf_.get_x().pos.allFinite()&&n->kf_.get_P().allFinite());
 auto noiseNode=node();auto noisy=group(60.,60.04,{60.004,60.008,60.012,60.016,60.020,60.024,60.028,60.032,60.036,60.04});
 for(int j=0;j<10;j++){auto mutableImu=std::const_pointer_cast<sensor_msgs::msg::Imu>(noisy.imu[j]);mutableImu->angular_velocity.x=j*1e-5;mutableImu->angular_velocity.y=0;mutableImu->angular_velocity.z=0;}
 call(*noiseNode,noisy,out);double sampleM2=0;for(int j=0;j<10;j++)sampleM2+=std::pow(j*1e-5-4.5e-5,2);
 check("actual_noisy_init_uses_unbiased_sample_mean_covariance",std::abs(noiseNode->kf_.get_P()(15,15)-sampleM2/(10*9))<1e-20);
 check("actual_noisy_init_mean_bias_preserved",std::abs(noiseNode->kf_.get_x().bg.x()-4.5e-5)<1e-16);
 auto n2=node();auto shortInit=group(30.,30.012,{30.004,30.008,30.012});call(*n2,shortInit,out);check("partial_init_no_valid_anchor",n2->imu_processor_->imu_need_init_&&!n2->imu_processor_->propagation_anchor_valid_);auto empty=group(30.014,30.020,{});auto state2=n2->kf_.get_x();call(*n2,empty,out);check("empty_IMU_does_not_change_init_state",n2->imu_processor_->init_iter_num==4&&(n2->kf_.get_x().bg-state2.bg).norm()==0.);
 // Valid scan end coincides with last input: next scan must still propagate only new time.
 auto n3=node();boot(*n3,40.,.04);auto aligned=group(40.04,40.052,{40.044,40.048,40.052},.2);call(*n3,aligned,out);check("aligned_boundary_velocity",std::abs(n3->kf_.get_x().vel.x()-.0020)<1e-12);check("aligned_boundary_position",std::abs(n3->kf_.get_x().pos.x()-.0000064)<1e-12);
 }
};
int PredictionProbe::checks=0;int PredictionProbe::failures=0;
int main(int argc,char**argv){rclcpp::init(argc,argv);PredictionProbe::run();rclcpp::shutdown();std::cout<<"{\"checks\":"<<PredictionProbe::checks<<",\"failures\":"<<PredictionProbe::failures<<"}\n";return PredictionProbe::failures?2:0;}
