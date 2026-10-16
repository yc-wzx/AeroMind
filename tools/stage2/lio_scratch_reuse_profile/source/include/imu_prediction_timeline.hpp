#pragma once
#include <optional>
#include <deque>
#include <vector>
#include <string>
#include <cmath>

// Original samples are immutable. State time is separate from sensor time.
// Only high-rate prediction is copied/replayed; the main mapping KF is not edited.
class ImuPredictionTimeline {
 public:
  using Filter = esekfom::esekf<state_ikfom, 12, input_ikfom>;
  static constexpr int64_t MAX_PAIR_GAP_NS = 200000000;
  static constexpr size_t MAX_HISTORY = 2048;
  struct Step { int64_t begin_ns, end_ns, head_ns, tail_ns; state_ikfom state_after; };
  static int64_t stamp(const sensor_msgs::msg::Imu &msg) { return rclcpp::Time(msg.header.stamp).nanoseconds(); }
  static bool valid_sample(const sensor_msgs::msg::Imu &m) {
    return stamp(m)>=0 && std::isfinite(m.angular_velocity.x) && std::isfinite(m.angular_velocity.y) && std::isfinite(m.angular_velocity.z) &&
      std::isfinite(m.linear_acceleration.x) && std::isfinite(m.linear_acceleration.y) && std::isfinite(m.linear_acceleration.z);
  }
  bool ready() const { return filter_.has_value() && time_ns_.has_value(); }
  int64_t time_ns() const { return time_ns_.value(); }
  const Filter &filter() const { return filter_.value(); }
  const std::string &reason() const { return reason_; }
  const std::vector<Step> &steps() const { return steps_; }
  const std::deque<sensor_msgs::msg::Imu> &history() const { return history_; }
  void invalidate(const std::string &why) { filter_.reset();time_ns_.reset();steps_.clear();reason_=why; }
  void clear() { history_.clear();invalidate("WAITING_CORRECTION"); }
  bool append(const sensor_msgs::msg::Imu &msg, ImuProcess &process) {
    steps_.clear();
    if (!valid_sample(msg)) { invalidate("NONFINITE_OR_INVALID_IMU"); return false; }
    int64_t t=stamp(msg);
    if (!history_.empty() && t<=stamp(history_.back())) { invalidate("DUPLICATE_OR_BACKWARD_IMU"); return false; }
    bool gap=!history_.empty() && t-stamp(history_.back())>MAX_PAIR_GAP_NS;
    history_.push_back(msg);while(history_.size()>MAX_HISTORY)history_.pop_front();
    if (gap) { invalidate("IMU_GAP"); return false; }
    if (!ready()) return true;
    if (history_.size()<2) { invalidate("NO_PAIR");return false; }
    const auto &head=history_[history_.size()-2];
    if (time_ns_.value()<stamp(head) || time_ns_.value()>=t) { invalidate("STATE_PAIR_TIME_MISMATCH");return false; }
    Filter candidate=filter_.value();
    try {
      auto state=process.IntegrateIMUInterval({head,msg},time_ns_.value(),candidate);
      if(!finite(candidate))throw std::runtime_error("nonfinite prediction state");
      steps_.push_back({time_ns_.value(),t,stamp(head),t,state});filter_=candidate;time_ns_=t;reason_.clear();return true;
    } catch(const std::exception &e) { invalidate(e.what());return false; }
  }
  bool reset(const Filter &corrected, int64_t corrected_time_ns, ImuProcess &process) {
    invalidate("WAITING_CORRECTION");
    if (!finite(corrected)) { reason_="NONFINITE_CORRECTED_STATE";return false; }
    if (history_.empty()) { reason_="NO_ORIGINAL_IMU_HISTORY";return false; }
    if (corrected_time_ns<stamp(history_.front()) || corrected_time_ns>stamp(history_.back())) { reason_="CORRECTION_NOT_BRACKETED";return false; }
    size_t anchor=0;
    while(anchor+1<history_.size() && stamp(history_[anchor+1])<=corrected_time_ns)++anchor;
    Filter candidate=corrected;int64_t current=corrected_time_ns;std::vector<Step> steps;
    try {
      for(size_t i=anchor+1;i<history_.size();++i) {
        int64_t head=stamp(history_[i-1]),tail=stamp(history_[i]);
        if(tail<=current)continue;
        if(head>current || tail-head>MAX_PAIR_GAP_NS)throw std::runtime_error("REPLAY_MISSING_IMU_INTERVAL");
        auto state=process.IntegrateIMUInterval({history_[i-1],history_[i]},current,candidate);
        if(!finite(candidate))throw std::runtime_error("nonfinite replay state");
        steps.push_back({current,tail,head,tail,state});current=tail;
      }
    } catch(const std::exception &e) { reason_=e.what();return false; }
    filter_=candidate;time_ns_=current;steps_=std::move(steps);reason_.clear();
    for(size_t i=0;i<anchor;++i)history_.pop_front();return true;
  }
 private:
  static bool finite(const Filter &f) { auto s=f.get_x();auto q=s.rot.coeffs();V3D g=s.grav;
    return s.pos.allFinite() && s.vel.allFinite() && s.ba.allFinite() && s.bg.allFinite() && g.allFinite() && q.allFinite() && std::abs(q.norm()-1.)<.001 && f.get_P().allFinite(); }
  std::optional<Filter> filter_;std::optional<int64_t> time_ns_;
  std::deque<sensor_msgs::msg::Imu> history_;std::vector<Step> steps_;std::string reason_="WAITING_CORRECTION";
};
