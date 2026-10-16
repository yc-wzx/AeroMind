#include "spark_fast_lio.h"
#include <new>
#include <array>
#include <cstring>
#include <iostream>
#include <iomanip>
struct PredictionProbe {
 static bool run(unsigned char pattern){
  alignas(ImuProcess) std::array<unsigned char,sizeof(ImuProcess)> storage;storage.fill(pattern);auto *p=new(storage.data()) ImuProcess;
  auto bytes=[&](const char*stage){std::array<unsigned char,sizeof(double)>end;std::array<unsigned char,3*sizeof(double)>acc;std::memcpy(end.data(),&p->last_lidar_end_time_,end.size());std::memcpy(acc.data(),p->acc_s_last.data(),acc.size());bool end_same=std::all_of(end.begin(),end.end(),[&](unsigned char x){return x==pattern;});bool acc_same=std::all_of(acc.begin(),acc.end(),[&](unsigned char x){return x==pattern;});std::cout<<"{\"stage\":\""<<stage<<"\",\"pattern\":"<<unsigned(pattern)<<",\"last_end_constructor_left_untouched\":"<<(end_same?"true":"false")<<",\"acc_cache_constructor_left_untouched\":"<<(acc_same?"true":"false")<<",\"last_end_bytes_hex\":\"";for(auto x:end)std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(x);std::cout<<std::dec<<"\",\"acc_cache_bytes_hex\":\"";for(auto x:acc)std::cout<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(x);std::cout<<std::dec<<"\"}\n";return p->last_lidar_end_time_==-1.0 && p->acc_s_last.isZero() && !p->propagation_anchor_valid_;};
  bool a=bytes("AFTER_ACTUAL_CONSTRUCTOR");p->last_lidar_end_time_=900.;p->acc_s_last=V3D::Constant(7.);p->propagation_anchor_valid_=true;p->Reset();bool b=bytes("AFTER_ACTUAL_RESET");p->~ImuProcess();return a&&b;
 }
};
int main(){bool a=PredictionProbe::run(0x5a),b=PredictionProbe::run(0xa5);std::cout<<"{\"checks\":8,\"startup_caches_initialized_and_anchor_invalid\":"<<(a&&b?"true":"false")<<",\"scope\":\"Object-representation bytes only; no typed read of indeterminate doubles, no filter prediction, no inference of unique replay root cause.\"}\n";return a&&b?0:2;}
