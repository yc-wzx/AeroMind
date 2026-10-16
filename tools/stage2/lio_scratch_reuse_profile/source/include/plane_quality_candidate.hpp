#pragma once
#include <Eigen/Eigen>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
namespace aeromind_plane {
// Isolated candidate experiment only; this is not an arena/navigation threshold.
inline constexpr double minimum_second_to_first_spread = 0.01;
struct Quality { bool finite=false; bool pass=false; double ratio=std::numeric_limits<double>::quiet_NaN(); };
template<class Points> Quality rank_quality(const Points& points) {
  Quality q; if(points.size()!=5)return q;
  Eigen::Vector3d mean=Eigen::Vector3d::Zero();
  for(const auto&p:points){Eigen::Vector3d v(p.x,p.y,p.z);if(!v.allFinite())return q;mean+=v;}mean/=5.;
  Eigen::Matrix3d cov=Eigen::Matrix3d::Zero();
  for(const auto&p:points){Eigen::Vector3d d=Eigen::Vector3d(p.x,p.y,p.z)-mean;cov+=d*d.transpose();}cov/=5.;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
  if(solver.info()!=Eigen::Success || !solver.eigenvalues().allFinite())return q;
  const auto e=solver.eigenvalues();if(!(e[2]>0.) || e[1]<0.)return q;
  q.ratio=e[1]/e[2];q.finite=std::isfinite(q.ratio);q.pass=q.finite && q.ratio>=minimum_second_to_first_spread;return q;
}
inline bool enabled(){static bool value=[](){const char*p=std::getenv("AEROMIND_PLANE_QUALITY_GUARD");return p && std::strcmp(p,"1")==0;}();return value;}
inline double lidar_stamp=-1;
struct Observation {bool fitted=false;bool legacy_selected=false;Quality quality;Eigen::Vector4f plane=Eigen::Vector4f::Zero();};
template<class Body,class Near,class Selected,class State> void record_first(const Body&body,const Near&near,const Selected&selected,const State&state,const std::vector<Observation>&obs,int count){
 static const char*path=std::getenv("AEROMIND_PLANE_QUALITY_TRACE");if(!path)return;
 static std::ofstream out(path);static double last=-1.;if(last==lidar_stamp)return;last=lidar_stamp;
 int fitted=0,legacy=0,rejected=0;for(const auto&o:obs){fitted+=o.fitted;legacy+=o.legacy_selected;rejected+=o.legacy_selected && !o.quality.pass;}
 out<<std::setprecision(17)<<"{\"stamp\":"<<lidar_stamp<<",\"guard_enabled\":"<<(enabled()?"true":"false")<<",\"ratio_threshold\":"<<minimum_second_to_first_spread<<",\"input_features\":"<<obs.size()<<",\"fitted\":"<<fitted<<",\"legacy_selected\":"<<legacy<<",\"rank_rejected_of_legacy\":"<<rejected<<",\"selected_count\":"<<count<<",\"state_position\":["<<state.pos[0]<<","<<state.pos[1]<<","<<state.pos[2]<<"],\"state_quaternion\":["<<state.rot.coeffs()[0]<<","<<state.rot.coeffs()[1]<<","<<state.rot.coeffs()[2]<<","<<state.rot.coeffs()[3]<<"],\"associations\":[";
 bool first=true;for(size_t i=0;i<obs.size();++i){const auto&o=obs[i];if(!o.fitted)continue;if(!first)out<<",";first=false;const auto&b=body.points[i];out<<"{\"feature_index\":"<<i<<",\"legacy_selected\":"<<(o.legacy_selected?"true":"false")<<",\"actual_selected\":"<<(selected[i]?"true":"false")<<",\"rank_pass\":"<<(o.quality.pass?"true":"false")<<",\"ratio\":";if(o.quality.finite)out<<o.quality.ratio;else out<<"null";out<<",\"body\":["<<b.x<<","<<b.y<<","<<b.z<<"],\"plane\":["<<o.plane[0]<<","<<o.plane[1]<<","<<o.plane[2]<<","<<o.plane[3]<<"],\"nearest\":[";bool fn=true;for(const auto&p:near[i]){if(!fn)out<<",";fn=false;out<<"["<<p.x<<","<<p.y<<","<<p.z<<"]";}out<<"]}";}out<<"]}\n";
}
}
