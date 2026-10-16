#pragma once
// Isolated, default-off, immutable snapshots. No JSON or disk IO under producer lock.
#include <array>
#include <algorithm>
#include <stdexcept>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>
namespace aeromind_input {
inline int64_t now_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
struct Sample {int64_t stamp_ns=0;std::string frame;std::array<double,37> values{};};
// values layout: quaternion xyzw, gyro xyz, accel xyz, then orientation/gyro/accel covariance (9 each).
struct Event {
 std::string kind;uint64_t sequence=0;int64_t entry_ns=0,capture_done_ns=0,enqueued_ns=0;
 double lidar_begin=-1,lidar_end=-1,last_lidar_end=-1;int64_t latest_received_imu_ns=-1;
 size_t imu_buffer_size=0,lidar_buffer_size=0;int init_count=-1;bool needs_init=false,first_init=false;
 std::vector<Sample> samples;std::vector<double> state,covariance,statistics;
 size_t bytes()const{size_t n=sizeof(Event)+kind.capacity()+samples.capacity()*sizeof(Sample)+(state.capacity()+covariance.capacity()+statistics.capacity())*sizeof(double);for(const auto&s:samples)n+=s.frame.capacity();return n;}
};
inline void quoted(std::ostream&o,const std::string&s){o<<'"';for(unsigned char c:s){if(c=='"'||c=='\\')o<<'\\'<<c;else if(c<32)o<<"\\u00"<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(c)<<std::dec<<std::setfill(' ');else o<<c;}o<<'"';}
inline void number(std::ostream&o,double x,uint64_t&invalid){if(!std::isfinite(x)){o<<"null";++invalid;}else {o<<std::setprecision(17)<<x;if(x==0&&std::signbit(x))o<<".0";}}
template<class T>inline void array(std::ostream&o,const T&a,uint64_t&invalid){o<<'[';bool first=true;for(double v:a){if(!first)o<<',';first=false;number(o,v,invalid);}o<<']';}
inline std::string serialize(const Event&e,uint64_t&invalid){std::ostringstream o;o<<"{\"format\":1,\"sequence\":"<<e.sequence<<",\"kind\":";quoted(o,e.kind);o<<",\"entry_ns\":"<<e.entry_ns<<",\"capture_done_ns\":"<<e.capture_done_ns<<",\"enqueued_ns\":"<<e.enqueued_ns<<",\"lidar_begin\":";number(o,e.lidar_begin,invalid);o<<",\"lidar_end\":";number(o,e.lidar_end,invalid);o<<",\"last_lidar_end\":";number(o,e.last_lidar_end,invalid);o<<",\"latest_received_imu_ns\":"<<e.latest_received_imu_ns<<",\"imu_buffer_size\":"<<e.imu_buffer_size<<",\"lidar_buffer_size\":"<<e.lidar_buffer_size<<",\"init_count\":"<<e.init_count<<",\"needs_init\":"<<(e.needs_init?"true":"false")<<",\"first_init\":"<<(e.first_init?"true":"false")<<",\"samples\":[";bool first=true;for(const auto&s:e.samples){if(!first)o<<',';first=false;o<<"{\"stamp_ns\":"<<s.stamp_ns<<",\"frame\":";quoted(o,s.frame);o<<",\"values\":";array(o,s.values,invalid);o<<'}';}o<<"],\"state\":";array(o,e.state,invalid);o<<",\"covariance\":";array(o,e.covariance,invalid);o<<",\"statistics\":";array(o,e.statistics,invalid);o<<"}\n";return o.str();}
class Writer {
 public:
 static constexpr size_t MAX_EVENTS=32768,MAX_BYTES=64*1024*1024,MAX_EVENT_BYTES=1024*1024;
 explicit Writer(const char*path):configured_(path&&*path),path_(configured_?path:""){
  if(!configured_)return;
  fd_=::open(path_.c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);
  if(fd_<0){++errors_;std::cerr<<"INPUT_TRACE_OPEN_FAILED: "<<std::strerror(errno)<<'\n';return;}
  health_fd_=::open((path_+".health.json").c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);
  if(health_fd_<0){++errors_;::close(fd_);fd_=-1;std::cerr<<"INPUT_TRACE_HEALTH_OPEN_FAILED\n";return;}
  try{worker_=std::thread([this]{work();});}catch(...){++errors_;::close(fd_);fd_=-1;}
 }
 ~Writer(){close();}
 bool enabled()const{return configured_;}
 void capture_error()noexcept{++capture_errors_;++received_;++dropped_;}
 void submit(Event e)noexcept{
  if(!configured_)return;
  ++received_;
  try{const size_t bytes=e.bytes();std::lock_guard<std::mutex>lk(mutex_);
   if(stopping_||fd_<0||bytes>MAX_EVENT_BYTES||queue_.size()>=MAX_EVENTS||queued_bytes_+bytes>MAX_BYTES){++dropped_;return;}
   const uint64_t next=admitted_+1;e.sequence=next;e.enqueued_ns=now_ns();queue_.push_back(std::move(e));admitted_=next;queued_bytes_+=bytes;peak_events_=std::max(peak_events_,queue_.size());peak_bytes_=std::max(peak_bytes_,queued_bytes_);cv_.notify_one();
  }catch(...){++capture_errors_;++dropped_;}
 }
 void close()noexcept{
  if(!configured_)return;
  {std::lock_guard<std::mutex>lk(mutex_);if(closed_)return;stopping_=true;cv_.notify_all();}
  if(worker_.joinable())worker_.join();
  if(fd_>=0){if(::fsync(fd_)!=0)++errors_;if(::close(fd_)!=0)++errors_;fd_=-1;}
  closed_=true;
  try{std::ostringstream o;o<<"{\"format\":1,\"received\":"<<received_<<",\"admitted\":"<<admitted_<<",\"written\":"<<written_<<",\"dropped\":"<<dropped_<<",\"errors\":"<<errors_<<",\"capture_errors\":"<<capture_errors_<<",\"nonfinite_fields\":"<<invalid_<<",\"closed\":true,\"queue_remaining\":"<<queue_.size()<<",\"peak_events\":"<<peak_events_<<",\"peak_bytes\":"<<peak_bytes_<<",\"max_events\":"<<MAX_EVENTS<<",\"max_bytes\":"<<MAX_BYTES<<",\"serialize_ns\":"<<serialize_ns_<<",\"write_ns\":"<<write_ns_<<"}\n";const auto s=o.str();if(health_fd_>=0){if(!write_all(health_fd_,s)||::fsync(health_fd_)!=0)std::cerr<<"INPUT_TRACE_HEALTH_WRITE_FAILED\n";if(::close(health_fd_)!=0)std::cerr<<"INPUT_TRACE_HEALTH_CLOSE_FAILED\n";health_fd_=-1;}std::cerr<<"INPUT_TRACE_CLOSED "<<s;}catch(...){std::cerr<<"INPUT_TRACE_CLOSE_FAILED\n";}
 }
 private:
 static bool write_all(int fd,const std::string&s){size_t n=0;while(n<s.size()){ssize_t w=::write(fd,s.data()+n,s.size()-n);if(w<0&&errno==EINTR)continue;if(w<=0)return false;n+=size_t(w);}return true;}
 void work()noexcept{
  for(;;){std::vector<Event>batch;try{std::unique_lock<std::mutex>lk(mutex_);cv_.wait(lk,[this]{return stopping_||!queue_.empty();});if(queue_.empty()&&stopping_)break;batch.reserve(64);while(!queue_.empty()&&batch.size()<64){queued_bytes_-=queue_.front().bytes();batch.push_back(std::move(queue_.front()));queue_.pop_front();}}catch(...){++errors_;return;}
   for(const auto&e:batch){try{auto t=now_ns();auto s=serialize(e,invalid_);serialize_ns_+=now_ns()-t;t=now_ns();
#ifdef AEROMIND_INPUT_TEST_FAULT
    bool ok=written_<3&&write_all(fd_,s);
#else
    bool ok=write_all(fd_,s);
#endif
    write_ns_+=now_ns()-t;if(ok)++written_;else ++errors_;
   }catch(...){++errors_;}}
  }
 }
 bool configured_=false,stopping_=false,closed_=false;std::string path_;int fd_=-1,health_fd_=-1;
 std::thread worker_;std::mutex mutex_;std::condition_variable cv_;std::deque<Event>queue_;size_t queued_bytes_=0,peak_bytes_=0,peak_events_=0;
 std::atomic<uint64_t>received_{0},dropped_{0},capture_errors_{0};uint64_t admitted_=0,written_=0,errors_=0,invalid_=0,serialize_ns_=0,write_ns_=0;
};
inline Writer& writer(){static Writer w(std::getenv("AEROMIND_INPUT_TRACE"));return w;}
template<class F>inline void capture(const char*kind,F&&snapshot)noexcept{auto&w=writer();if(!w.enabled())return;try{Event e;e.kind=kind;e.entry_ns=now_ns();snapshot(e);e.capture_done_ns=now_ns();w.submit(std::move(e));}catch(...){w.capture_error();}}
// Default-off timing: statistics are integer-valued monotonic nanoseconds, then raw message stamp.
// order: callback_start, lock_wait_start, lock_acquired, locked_scope_end, callback_end, message_stamp_ns.
// Locked scope end is sampled immediately before lock_guard destruction, not claimed as exact unlock time.
class CallbackTiming {
 public:
 CallbackTiming(const char*kind,int64_t stamp=-1):kind_(kind),stamp_(stamp),enabled_(writer().enabled()){if(enabled_)start_=now_ns();}
 void wait(){if(enabled_)wait_=now_ns();} void acquired(){if(enabled_)acquired_=now_ns();} void locked_end(){if(enabled_)locked_end_=now_ns();}
 ~CallbackTiming(){if(!enabled_)return;const auto end=now_ns();capture(kind_,[&](auto&e){e.statistics={double(start_),double(wait_),double(acquired_),double(locked_end_),double(end),double(stamp_)};});}
 private:const char*kind_;int64_t stamp_,start_=0,wait_=0,acquired_=0,locked_end_=0;bool enabled_;
};
struct LockedTiming {CallbackTiming &timing;~LockedTiming(){timing.locked_end();}};
// Deep copy every original IMU field; source objects and filter state are never changed.
template<class Msg>inline Sample sample(const Msg&m){Sample s;s.stamp_ns=int64_t(m.header.stamp.sec)*1000000000LL+m.header.stamp.nanosec;s.frame=m.header.frame_id;size_t i=0;for(double x:{m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z})s.values[i++]=x;for(const auto*a:{&m.orientation_covariance,&m.angular_velocity_covariance,&m.linear_acceleration_covariance})for(double x:*a)s.values[i++]=x;return s;}
template<class M>inline void samples(Event&e,const M&messages){if(messages.size()>4096)throw std::length_error("input snapshot sample bound");e.samples.reserve(messages.size());for(const auto&m:messages)e.samples.push_back(sample(*m));}
// state layout: pos3, rot row-major9, vel3, ba3, bg3, gravity3, offset_T3, offset_R row-major9 = 36.
template<class K>inline void filter(Event&e,K&kf){const auto s=kf.get_x();auto vec=[&](const auto&v){for(int i=0;i<3;i++)e.state.push_back(v[i]);};auto mat=[&](const auto&m){for(int i=0;i<3;i++)for(int j=0;j<3;j++)e.state.push_back(m(i,j));};vec(s.pos);mat(s.rot.toRotationMatrix());vec(s.vel);vec(s.ba);vec(s.bg);vec(s.grav.vec);vec(s.offset_T_L_I);mat(s.offset_R_L_I.toRotationMatrix());const auto p=kf.get_P();for(int i=0;i<p.rows();i++)for(int j=0;j<p.cols();j++)e.covariance.push_back(p(i,j));}
}
