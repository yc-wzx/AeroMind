#pragma once
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>
#include <algorithm>
namespace aeromind_lineage {
// Diagnostic only. Tree algorithms and ROS publishing never read these records.
struct Context {const char* stage="UNSET";double stamp=-1.;int feature=-1;std::array<double,3> pos{};std::array<double,4> quat{{0,0,0,1}};};
inline thread_local Context context;
inline uint64_t monotonic_ns(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
inline constexpr const char* timing_kinds[]={"BUILD_BEGIN","BUILD_COMPLETE","QUERY","VOXEL_DECISION","DIRECT_ADD_COMPLETE","PROPOSE","DELETE_BOXES_COMPLETE","DELETE_POINTS_COMPLETE","RESTORE_BOXES_COMPLETE"};
struct Timing {uint64_t count=0,wait_sum_ns=0,wait_max_ns=0,work_sum_ns=0,work_max_ns=0,work_ge_1ms=0,capture_sum_ns=0,capture_max_ns=0;};
struct Point {float x=0,y=0,z=0;};
struct Box {std::array<double,3> min{},max{};};
struct OwnedContext {std::string stage;double stamp=-1;int feature=-1;std::array<double,3> pos{};std::array<double,4> quat{};};
struct Event {
 uint64_t seq=0,entry=0,capture_done=0,queue_entry=0,acquired=0,tree=0;size_t kind=0;OwnedContext ctx;
 Point query{},input{},winner{},world{},body{};std::vector<Point> points,neighbors,existing;std::vector<float> distances;std::vector<Box> boxes;Box voxel_box{};
 int selected_existing=-1,source_feature=-1,root_branch=-1,deleted_count=0;bool committed=false,selected=false;
 size_t bytes()const{return sizeof(Event)+ctx.stage.capacity()+points.capacity()*sizeof(Point)+neighbors.capacity()*sizeof(Point)+existing.capacity()*sizeof(Point)+distances.capacity()*sizeof(float)+boxes.capacity()*sizeof(Box);}
};
struct Limits {size_t queue_events=65536,queue_bytes=64*1024*1024,event_bytes=16*1024*1024,batch_events=256,batch_bytes=1024*1024;};
#ifdef AEROMIND_LINEAGE_TESTING
struct TestFaults {bool pause_worker=false;int fail_after_batches=-1;};
inline TestFaults test_defaults;
#endif
inline void number(std::ostream&o,double v){if(std::isfinite(v)){if(v==0&&std::signbit(v))o<<"-0.0";else o<<v;}else o<<"null";}
inline void quoted(std::ostream&o,const std::string&s){o<<'"';for(unsigned char c:s){if(c=='"'||c=='\\')o<<'\\'<<char(c);else if(c<32){o<<"\\u00"<<"0123456789abcdef"[c>>4]<<"0123456789abcdef"[c&15];}else o<<char(c);}o<<'"';}
inline void point(std::ostream&o,const Point&p){uint32_t a,b,c;std::memcpy(&a,&p.x,4);std::memcpy(&b,&p.y,4);std::memcpy(&c,&p.z,4);o<<"{\"xyz\":[";number(o,p.x);o<<",";number(o,p.y);o<<",";number(o,p.z);o<<"],\"xyz_bits\":["<<a<<","<<b<<","<<c<<"]}";}
inline void points(std::ostream&o,const std::vector<Point>&p){o<<'[';for(size_t i=0;i<p.size();++i){if(i)o<<',';point(o,p[i]);}o<<']';}
inline void box(std::ostream&o,const Box&b){o<<"{\"min\":[";for(int i=0;i<3;++i){if(i)o<<',';number(o,b.min[i]);}o<<"],\"max\":[";for(int i=0;i<3;++i){if(i)o<<',';number(o,b.max[i]);}o<<"]}";}
inline std::string serialize(const Event&e){std::ostringstream o;o<<std::setprecision(17)<<"{\"seq\":"<<e.seq<<",\"entry_monotonic_ns\":"<<e.entry<<",\"capture_done_monotonic_ns\":"<<e.capture_done<<",\"queue_entry_monotonic_ns\":"<<e.queue_entry<<",\"lock_acquired_monotonic_ns\":"<<e.acquired<<",\"writer_monotonic_ns\":"<<monotonic_ns()<<",\"tree_token\":"<<e.tree<<",\"kind\":\""<<timing_kinds[e.kind]<<"\",\"context\":{\"stage\":";quoted(o,e.ctx.stage);o<<",\"stamp\":";number(o,e.ctx.stamp);o<<",\"feature_index\":"<<e.ctx.feature<<",\"position\":[";for(int i=0;i<3;++i){if(i)o<<',';number(o,e.ctx.pos[i]);}o<<"],\"quaternion\":[";for(int i=0;i<4;++i){if(i)o<<',';number(o,e.ctx.quat[i]);}o<<"]}";
 switch(e.kind){
 case 0:o<<",\"points\":";points(o,e.points);break;
 case 1:break;
 case 2:o<<",\"query\":";point(o,e.query);o<<",\"neighbors\":";points(o,e.neighbors);o<<",\"squared_distances\":[";for(size_t i=0;i<e.distances.size();++i){if(i)o<<',';number(o,e.distances[i]);}o<<']';break;
 case 3:o<<",\"input\":";point(o,e.input);o<<",\"winner\":";point(o,e.winner);o<<",\"box\":";box(o,e.voxel_box);o<<",\"existing\":";points(o,e.existing);o<<",\"selected_existing_index\":"<<e.selected_existing<<",\"committed\":"<<(e.committed?"true":"false")<<",\"root_rebuild_branch\":"<<e.root_branch;break;
 case 4:o<<",\"point\":";point(o,e.input);break;
 case 5:o<<",\"source_feature_index\":"<<e.source_feature<<",\"world\":";point(o,e.world);o<<",\"body\":";point(o,e.body);o<<",\"measurement_selected\":"<<(e.selected?"true":"false");break;
 case 6:case 8:o<<",\"boxes\":[";for(size_t i=0;i<e.boxes.size();++i){if(i)o<<',';box(o,e.boxes[i]);}o<<']';if(e.kind==6)o<<",\"reported_deleted_count\":"<<e.deleted_count;break;
 case 7:o<<",\"requested_points\":";points(o,e.points);break;
 }
 o<<"}\n";return o.str();
}
struct Writer {
 bool enabled=false;std::string path;Limits limits;std::atomic<uint64_t> received{0},written{0},errors{0},dropped{0};std::array<Timing,9> timing{};
 std::mutex mutex;std::condition_variable cv;std::deque<Event> queue;size_t queue_bytes=0,queue_peak_bytes=0,queue_peak_events=0,batch_peak_bytes=0,batch_peak_events=0;
 uint64_t admitted=0,batches=0,bytes_written=0,serialize_sum_ns=0,serialize_max_ns=0,write_sum_ns=0,write_max_ns=0,dequeue_sum_ns=0,dequeue_max_ns=0;
 bool closing=false;std::atomic<bool> finished{false},fatal{false};int fd=-1,health_fd=-1;std::thread worker;
#ifdef AEROMIND_LINEAGE_TESTING
 TestFaults faults;std::mutex gate_mutex;std::condition_variable gate_cv;
#endif
 Writer():Writer(std::getenv("AEROMIND_MAP_LINEAGE_FILE"),Limits{}){}
 explicit Writer(const char*p,Limits lim):limits(lim){
#ifdef AEROMIND_LINEAGE_TESTING
 faults=test_defaults;
#endif
 if(!p||!*p)return;enabled=true;
 try{path=p;if(!limits.queue_events||!limits.queue_bytes||!limits.event_bytes||!limits.batch_events||!limits.batch_bytes){++errors;fatal=true;return;}
 fd=::open(p,O_WRONLY|O_CREAT|O_EXCL,0600);if(fd<0){++errors;fatal=true;}
 health_fd=::open((path+".health.json").c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);if(health_fd<0){++errors;fatal=true;}
 if(!fatal.load())worker=std::thread([this]{run();});
 }catch(...){++errors;fatal=true;}
 }
 static bool put(int f,const std::string&s){size_t i=0;while(i<s.size()){ssize_t n=::write(f,s.data()+i,s.size()-i);if(n<0&&errno==EINTR)continue;if(n<=0)return false;i+=size_t(n);}return true;}
 void capture_failure(size_t kind,uint64_t entered)noexcept{try{std::lock_guard<std::mutex>lock(mutex);++received;++errors;++dropped;auto&t=timing[kind];++t.count;auto elapsed=monotonic_ns()-entered;t.capture_sum_ns+=elapsed;t.capture_max_ns=std::max(t.capture_max_ns,elapsed);}catch(...){++errors;}}
 void enqueue(Event&&e)noexcept{
  e.queue_entry=monotonic_ns();try{std::lock_guard<std::mutex>lock(mutex);e.acquired=monotonic_ns();e.seq=++received;auto&t=timing[e.kind];++t.count;auto wait=e.acquired-e.queue_entry;auto capture=e.capture_done-e.entry;t.wait_sum_ns+=wait;t.wait_max_ns=std::max(t.wait_max_ns,wait);t.capture_sum_ns+=capture;t.capture_max_ns=std::max(t.capture_max_ns,capture);
  size_t n=e.bytes();if(closing||fatal.load()||n>limits.event_bytes||n>limits.queue_bytes||queue.size()>=limits.queue_events||n>limits.queue_bytes-queue_bytes){++dropped;if(!fatal.load())++errors;}
  else{try{queue.push_back(std::move(e));queue_bytes+=n;++admitted;queue_peak_bytes=std::max(queue_peak_bytes,queue_bytes);queue_peak_events=std::max(queue_peak_events,queue.size());}catch(...){++dropped;++errors;}}
  auto work=monotonic_ns()-e.acquired;t.work_sum_ns+=work;t.work_max_ns=std::max(t.work_max_ns,work);t.work_ge_1ms+=work>=1000000;
  }catch(...){++errors;++dropped;}cv.notify_one();
 }
 void run()noexcept{
#ifdef AEROMIND_LINEAGE_TESTING
  {std::unique_lock<std::mutex>lock(gate_mutex);gate_cv.wait(lock,[this]{return !faults.pause_worker;});}
#endif
  try{std::vector<Event>batch;batch.reserve(limits.batch_events);while(true){batch.clear();size_t nbytes=0;const auto dequeue_entry=monotonic_ns();{
   std::unique_lock<std::mutex>lock(mutex);cv.wait(lock,[this]{return closing||!queue.empty();});if(queue.empty()&&closing)break;
   const auto acquired=monotonic_ns();while(!queue.empty()&&batch.size()<limits.batch_events){size_t n=queue.front().bytes();if(!batch.empty()&&n>limits.batch_bytes-std::min(nbytes,limits.batch_bytes))break;batch.push_back(std::move(queue.front()));queue.pop_front();queue_bytes-=n;nbytes+=n;}
   auto held=monotonic_ns()-acquired;dequeue_sum_ns+=held;dequeue_max_ns=std::max(dequeue_max_ns,held);
  }(void)dequeue_entry;batch_peak_bytes=std::max(batch_peak_bytes,nbytes);batch_peak_events=std::max(batch_peak_events,batch.size());
  if(fatal.load()){dropped+=batch.size();continue;}
  std::string buffer;const auto ser_start=monotonic_ns();try{for(const auto&e:batch)buffer+=serialize(e);}catch(...){++errors;fatal=true;dropped+=batch.size();continue;}
  const auto ser_done=monotonic_ns();serialize_sum_ns+=ser_done-ser_start;serialize_max_ns=std::max(serialize_max_ns,ser_done-ser_start);const auto write_start=monotonic_ns();bool ok;
#ifdef AEROMIND_LINEAGE_TESTING
  ok=!(faults.fail_after_batches>=0&&batches>=uint64_t(faults.fail_after_batches))&&put(fd,buffer);
#else
  ok=put(fd,buffer);
#endif
  const auto write_done=monotonic_ns();write_sum_ns+=write_done-write_start;write_max_ns=std::max(write_max_ns,write_done-write_start);++batches;
  if(ok){written+=batch.size();bytes_written+=buffer.size();}else{++errors;fatal=true;dropped+=batch.size();}
  }}catch(...){++errors;fatal=true;std::lock_guard<std::mutex>lock(mutex);dropped+=queue.size();queue.clear();queue_bytes=0;}
 }
#ifdef AEROMIND_LINEAGE_TESTING
 void release_worker(){std::lock_guard<std::mutex>lock(gate_mutex);faults.pause_worker=false;gate_cv.notify_all();}
#endif
 // Caller must quiesce producers before finish. Normal process exit does this after node teardown.
 void finish()noexcept{if(finished.exchange(true)||!enabled)return;try{{std::lock_guard<std::mutex>lock(mutex);closing=true;}cv.notify_all();
#ifdef AEROMIND_LINEAGE_TESTING
 release_worker();
#endif
 if(worker.joinable())worker.join();if(fd>=0){if(::fsync(fd)!=0)++errors;if(::close(fd)!=0)++errors;fd=-1;}
 std::ostringstream h;h<<"{\"format_version\":2,\"received\":"<<received.load()<<",\"admitted\":"<<admitted<<",\"written\":"<<written.load()<<",\"dropped\":"<<dropped.load()<<",\"errors\":"<<errors.load()<<",\"closed\":true,\"queue_remaining\":"<<queue.size()<<",\"queue_remaining_bytes\":"<<queue_bytes<<",\"queue_peak_events\":"<<queue_peak_events<<",\"queue_peak_bytes\":"<<queue_peak_bytes<<",\"batch_peak_events\":"<<batch_peak_events<<",\"batch_peak_bytes\":"<<batch_peak_bytes<<",\"limits\":{\"queue_events\":"<<limits.queue_events<<",\"queue_bytes\":"<<limits.queue_bytes<<",\"event_bytes\":"<<limits.event_bytes<<",\"batch_events\":"<<limits.batch_events<<",\"batch_bytes\":"<<limits.batch_bytes<<"},\"worker\":{\"batches\":"<<batches<<",\"bytes_written\":"<<bytes_written<<",\"serialize_sum_ns\":"<<serialize_sum_ns<<",\"serialize_max_ns\":"<<serialize_max_ns<<",\"write_sum_ns\":"<<write_sum_ns<<",\"write_max_ns\":"<<write_max_ns<<",\"dequeue_lock_sum_ns\":"<<dequeue_sum_ns<<",\"dequeue_lock_max_ns\":"<<dequeue_max_ns<<"},\"timing_semantics\":\"capture before queue; wait is queue mutex wait; work is producer enqueue critical section; worker serialization/write separately measured\",\"timing\":{";
 for(size_t i=0;i<timing.size();++i){if(i)h<<',';auto&t=timing[i];h<<'"'<<timing_kinds[i]<<"\":{\"count\":"<<t.count<<",\"wait_sum_ns\":"<<t.wait_sum_ns<<",\"wait_max_ns\":"<<t.wait_max_ns<<",\"work_sum_ns\":"<<t.work_sum_ns<<",\"work_max_ns\":"<<t.work_max_ns<<",\"work_ge_1ms\":"<<t.work_ge_1ms<<",\"capture_sum_ns\":"<<t.capture_sum_ns<<",\"capture_max_ns\":"<<t.capture_max_ns<<"}";}h<<"}}\n";
 if(health_fd>=0){if(!put(health_fd,h.str())||::fsync(health_fd)!=0)++errors;if(::close(health_fd)!=0)++errors;health_fd=-1;}
 std::fprintf(stderr,"lineage_async_close received=%llu written=%llu dropped=%llu errors=%llu\n",(unsigned long long)received.load(),(unsigned long long)written.load(),(unsigned long long)dropped.load(),(unsigned long long)errors.load());
 }catch(...){++errors;std::fprintf(stderr,"lineage_async_close_exception errors=%llu\n",(unsigned long long)errors.load());}}
 ~Writer(){finish();}
};
inline Writer& writer(){static Writer w;return w;}
inline bool active(){return writer().enabled;}
template<class P>Point snapshot_point(const P&p){return Point{float(p.x),float(p.y),float(p.z)};}
template<class PS>std::vector<Point> snapshot_points(const PS&ps){if(ps.size()>16*1024*1024/sizeof(Point))throw std::length_error("diagnostic point snapshot too large");std::vector<Point>v;v.reserve(ps.size());for(const auto&p:ps)v.push_back(snapshot_point(p));return v;}
template<class B>Box snapshot_box(const B&b){Box v;for(int i=0;i<3;++i){v.min[i]=b.vertex_min[i];v.max[i]=b.vertex_max[i];}return v;}
template<class BS>std::vector<Box> snapshot_boxes(const BS&bs){if(bs.size()>16*1024*1024/sizeof(Box))throw std::length_error("diagnostic box snapshot too large");std::vector<Box>v;v.reserve(bs.size());for(const auto&b:bs)v.push_back(snapshot_box(b));return v;}
template<class S>void set_context(const char*stage,double stamp,const S&s,int feature=-1)noexcept{if(!active())return;context.stage=stage;context.stamp=stamp;context.feature=feature;for(int i=0;i<3;++i)context.pos[i]=s.pos[i];for(int i=0;i<4;++i)context.quat[i]=s.rot.coeffs()[i];}
inline void fixture_context(const char*stage,double stamp,int feature=-1){context.stage=stage;context.stamp=stamp;context.feature=feature;}
template<class FN>void capture(const void*t,size_t kind,FN fill)noexcept{auto&w=writer();if(!w.enabled)return;auto entered=monotonic_ns();try{Event e;e.kind=kind;e.entry=entered;e.tree=uintptr_t(t);if(context.stage&&std::strlen(context.stage)>1024)throw std::length_error("diagnostic stage too large");e.ctx.stage=context.stage?context.stage:"";e.ctx.stamp=context.stamp;e.ctx.feature=context.feature;e.ctx.pos=context.pos;e.ctx.quat=context.quat;fill(e);e.capture_done=monotonic_ns();w.enqueue(std::move(e));}catch(...){w.capture_failure(kind,entered);}}
template<class PS>void build_begin(const void*t,const PS&ps){capture(t,0,[&](Event&e){e.points=snapshot_points(ps);});}
inline void build_end(const void*t){capture(t,1,[](Event&){});}
template<class P,class PS>void query(const void*t,const P&p,const PS&ps,const std::vector<float>&dist){capture(t,2,[&](Event&e){e.query=snapshot_point(p);e.neighbors=snapshot_points(ps);if(dist.size()>16*1024*1024/sizeof(float))throw std::length_error("diagnostic distance snapshot too large");e.distances=dist;});}
template<class P,class B,class PS>void voxel(const void*t,const P&input,const P&winner,const B&b,const PS&existing,int selected_existing,bool committed,int root_branch){capture(t,3,[&](Event&e){e.input=snapshot_point(input);e.winner=snapshot_point(winner);e.voxel_box=snapshot_box(b);e.existing=snapshot_points(existing);e.selected_existing=selected_existing;e.committed=committed;e.root_branch=root_branch;});}
template<class P>void add_direct(const void*t,const P&p){capture(t,4,[&](Event&e){e.input=snapshot_point(p);});}
template<class P,class B>void proposal(const void*t,int feature,const P&world,const B&body,bool selected){capture(t,5,[&](Event&e){e.source_feature=feature;e.world=snapshot_point(world);e.body=snapshot_point(body);e.selected=selected;});}
template<class BS>void delete_boxes(const void*t,const BS&bs,int count){capture(t,6,[&](Event&e){e.boxes=snapshot_boxes(bs);e.deleted_count=count;});}
template<class PS>void delete_points(const void*t,const PS&ps){capture(t,7,[&](Event&e){e.points=snapshot_points(ps);});}
template<class BS>void restore_boxes(const void*t,const BS&bs){capture(t,8,[&](Event&e){e.boxes=snapshot_boxes(bs);});}
} // namespace
