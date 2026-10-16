#pragma once
#include <Eigen/Eigen>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <cstdint>
namespace robust_surface {
inline bool enabled(){static bool e=[](){const char*p=std::getenv("AEROMIND_LIO_CONSISTENCY_FIX");return p&&std::strcmp(p,"1")==0;}();return e;}
struct Fit{bool valid=false;Eigen::Vector4f plane;int count=0;double angular_std=0,rms=0,variance=0;};
inline bool tls(const std::vector<Eigen::Vector3d>&p,const std::vector<int>&ix,Eigen::Vector3d&n,double&d,double&angle,double&sse){
 if(ix.size()<8)return false;Eigen::Vector3d mean=Eigen::Vector3d::Zero();for(int i:ix)mean+=p[i];mean/=ix.size();Eigen::Matrix3d C=Eigen::Matrix3d::Zero();for(int i:ix){auto v=(p[i]-mean).eval();C+=v*v.transpose();}C/=ix.size();Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> sol(C);if(sol.info()!=Eigen::Success)return false;auto ev=sol.eigenvalues();if(!ev.allFinite()||ev[1]<=ev[0]||ev[2]<=ev[0])return false;
 angle=std::sqrt(.0001/ix.size()*(1/(ev[1]-ev[0])+1/(ev[2]-ev[0])));n=sol.eigenvectors().col(0);d=-n.dot(mean);sse=std::max(0.,ev[0])*ix.size();return n.allFinite()&&std::isfinite(d);
}
template<class Points> Fit fit(const Points&ps,const Eigen::Vector3d&q){
 Fit f;if(!q.allFinite())return f;std::vector<Eigen::Vector3d> p;p.reserve(ps.size());for(auto&v:ps){Eigen::Vector3d x(v.x,v.y,v.z);if(!x.allFinite())return f;if((x-q).squaredNorm()<=.25)p.push_back(x);}if(p.size()<8)return f;
 std::vector<int> best;best.reserve(p.size());std::vector<int> ix;ix.reserve(p.size());double bestscore=1e100;uint32_t rng=8675309;auto next=[&](){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return int(rng%p.size());};
 for(int trial=0;trial<48;trial++){int a=next(),b=next(),c=next();if(a==b||a==c||b==c)continue;Eigen::Vector3d n=(p[b]-p[a]).cross(p[c]-p[a]);if(n.norm()<1e-10)continue;n.normalize();double d=-n.dot(p[a]);if(std::abs(n.dot(q)+d)>.03)continue;
 ix.clear();double score=0;for(int i=0;i<int(p.size());i++){double e=n.dot(p[i])+d;if(std::abs(e)<=.03){ix.push_back(i);score+=e*e;}}
 if(ix.size()>best.size()||(ix.size()==best.size()&&score<bestscore)){best=ix;bestscore=score;}}
 Eigen::Vector3d n;double d,angle,sse;if(!tls(p,best,n,d,angle,sse))return f;
 std::vector<int> refined;refined.reserve(p.size());for(int i=0;i<int(p.size());i++)if(std::abs(n.dot(p[i])+d)<=.03)refined.push_back(i);
 if(!tls(p,refined,n,d,angle,sse))return f;double df=refined.size()-3,logalpha=std::log(1/.0027),limit=df+2*std::sqrt(df*logalpha)+2*logalpha;
 if(sse>limit*.0001||angle>.17453292519943295||std::abs(n.dot(q)+d)>.03)return f;
 // Conservative range-noise envelope .01m for query and retained map points.
 // Fit centroid/normal covariance from local inlier support, no GT or motion-state shortcut.
 Eigen::Vector3d mean=Eigen::Vector3d::Zero();for(int i:refined)mean+=p[i];mean/=refined.size();Eigen::Matrix3d C=Eigen::Matrix3d::Zero();for(int i:refined){auto delta=(p[i]-mean).eval();C+=delta*delta.transpose();}C/=refined.size();Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> sol(C);auto eigen=sol.eigenvalues();auto U=sol.eigenvectors();Eigen::Vector3d delta=q-mean;
 f.variance=.0001*(1.+1./refined.size())+.0001/refined.size()*(std::pow(delta.dot(U.col(1)),2)/(eigen[1]-eigen[0])+std::pow(delta.dot(U.col(2)),2)/(eigen[2]-eigen[0]));
 if(!std::isfinite(f.variance)||f.variance<=0)return Fit{};
 f.plane.head<3>()=n.cast<float>();f.plane[3]=float(d);f.valid=f.plane.allFinite();f.count=refined.size();f.angular_std=angle;f.rms=std::sqrt(sse/refined.size());return f;
}
}
