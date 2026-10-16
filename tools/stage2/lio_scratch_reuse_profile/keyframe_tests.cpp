#include "spark_fast_lio.h"
#include <iostream>
struct PredictionProbe {
 static int count,fail;static void ck(const char*n,bool b){count++;fail+=!b;std::cout<<n<<" "<<b<<"\n";}
 static void run(){rclcpp::NodeOptions o;o.parameter_overrides({rclcpp::Parameter("mapping.det_range",16.),rclcpp::Parameter("filter_size_map",.05),rclcpp::Parameter("common.base_frame",std::string("")),rclcpp::Parameter("common.planar_mode",false),rclcpp::Parameter("gravity_alignment.enable_gravity_alignment",false)});
 auto n=std::make_shared<spark_fast_lio::SPARKFastLIO2>(o);
 PointVector pts;for(int i=0;i<16;i++){PointType p{};p.x=2.;p.y=.05*(i%4);p.z=.05*(i/4);pts.push_back(p);}n->ikd_tree_.Build(pts);n->feats_down_size_=16;n->feats_down_body_->points=pts;n->feats_down_world_->resize(16);n->nearest_points_.resize(16);n->latest_state_=n->kf_.get_x();n->latest_state_.pos.setZero();
 n->mapIncremental();ck("first_call_creates_anchor",n->keyframe_map_anchor_valid_&&n->keyframe_map_position_.norm()==0);
 n->latest_state_.pos[0]=.02;n->mapIncremental();ck("subvoxel_reobserve_skipped",n->keyframe_map_position_.norm()==0);
 ck("skip_does_not_clamp_estimated_position",n->latest_state_.pos[0]==.02);
 n->latest_state_.pos[0]=.1;n->mapIncremental();ck("two_voxels_translation_inserts",n->keyframe_map_position_[0]==.1);
 n->latest_state_.pos[0]=.101;n->mapIncremental();ck("stationary_near_new_keyframe_skipped",n->keyframe_map_position_[0]==.1);
 const double angle=2.*n->filter_size_map_min_/n->det_range_*1.01;
 n->latest_state_.rot=SO3(Eigen::AngleAxisd(angle,Eigen::Vector3d::UnitZ()).toRotationMatrix());n->mapIncremental();ck("rotation_parallax_inserts",std::abs(Eigen::AngleAxisd(n->keyframe_map_rotation_).angle()-angle)<1e-12);
 ck("rotation_does_not_clamp_velocity",n->latest_state_.vel.norm()==0);
 n->latest_state_.pos[0]=.201;n->mapIncremental();ck("new_region_keeps_building_map",n->keyframe_map_position_[0]==.201);
 auto z=std::make_shared<spark_fast_lio::SPARKFastLIO2>(o);ck("anchor_is_per_node_not_process_static",!z->keyframe_map_anchor_valid_);
 }
};int PredictionProbe::count=0,PredictionProbe::fail=0;
int main(int argc,char**argv){rclcpp::init(argc,argv);PredictionProbe::run();rclcpp::shutdown();std::cout<<"checks="<<PredictionProbe::count<<" failures="<<PredictionProbe::fail<<"\n";return PredictionProbe::fail?2:0;}
