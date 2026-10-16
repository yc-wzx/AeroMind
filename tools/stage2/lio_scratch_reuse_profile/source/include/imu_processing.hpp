#include <cstdlib>
#include <cstring>
#include <math.h>

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <fstream>
#include <mutex>
#include <thread>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

// AEROMIND_INPUT_BEGIN
#include "input_bundle_trace.hpp"
// AEROMIND_INPUT_END
#include "common/common_lib.h"
#include "common/so3_math.h"
#include "common/use-ikfom.hpp"

/// *************Preconfiguration

#define MAX_INI_COUNT (10)

bool time_list(PointType &x, PointType &y) { return (x.curvature < y.curvature); }

/// *************IMU Process and undistortion
class ImuProcess {
  friend struct PredictionProbe;
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Reset();
  void Reset(double start_timestamp, const std::shared_ptr<const sensor_msgs::msg::Imu> &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4, 4) & T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas,
               esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
               PointCloudXYZI::Ptr pcl_un_);
  state_ikfom IntegrateIMU(const std::deque<sensor_msgs::msg::Imu> imu_queue,
                           esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state);
  state_ikfom IntegrateIMUInterval(const std::deque<sensor_msgs::msg::Imu> &queue, int64_t start_ns,
      esekfom::esekf<state_ikfom,12,input_ikfom> &kf_state);
  std::ofstream fout_imu;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double first_lidar_time;

 private:
  void IMU_init(const MeasureGroup &meas,
                esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                int &N);
  void UndistortPcl(const MeasureGroup &meas,
                    esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                    PointCloudXYZI &pcl_in_out);

  PointCloudXYZI::Ptr cur_pcl_un_;
  std::shared_ptr<const sensor_msgs::msg::Imu> last_imu_;
  std::deque<std::shared_ptr<const sensor_msgs::msg::Imu>> v_imu_;
  std::vector<Pose6D> IMUpose;
  std::vector<M3D> v_rot_pcl_;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double start_timestamp_;
  double last_lidar_end_time_;
// AEROMIND_STARTUP_FIX_BEGIN
  bool propagation_anchor_valid_ = false;
// AEROMIND_STARTUP_FIX_END
  int init_iter_num   = 1;
  bool b_first_frame_ = true;
  bool imu_need_init_ = true;
};

ImuProcess::ImuProcess() : start_timestamp_(-1), b_first_frame_(true), imu_need_init_(true) {
  init_iter_num   = 1;
  Q               = process_noise_cov();
  cov_acc         = V3D(0.1, 0.1, 0.1);
  cov_gyr         = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr    = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_acc    = V3D(0.0001, 0.0001, 0.0001);
  mean_acc        = V3D(0, 0, -1.0);
  mean_gyr        = V3D(0, 0, 0);
  angvel_last     = Zero3d;
// AEROMIND_STARTUP_FIX_BEGIN
  acc_s_last = Zero3d;
  last_lidar_end_time_ = -1.0;
// AEROMIND_STARTUP_FIX_END
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  last_imu_.reset(new sensor_msgs::msg::Imu());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() {
  // ROS_WARN("Reset ImuProcess");
  mean_acc         = V3D(0, 0, -1.0);
  mean_gyr         = V3D(0, 0, 0);
  angvel_last      = Zero3d;
// AEROMIND_STARTUP_FIX_BEGIN
  acc_s_last = Zero3d;
  last_lidar_end_time_ = -1.0;
  propagation_anchor_valid_ = false;
  b_first_frame_ = true;
// AEROMIND_STARTUP_FIX_END
  imu_need_init_   = true;
  start_timestamp_ = -1;
  init_iter_num    = 1;
  v_imu_.clear();
  IMUpose.clear();
  last_imu_.reset(new sensor_msgs::msg::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::set_extrinsic(const MD(4, 4) & T) {
  Lidar_T_wrt_IMU = T.block<3, 1>(0, 3);
  Lidar_R_wrt_IMU = T.block<3, 3>(0, 0);
}

void ImuProcess::set_extrinsic(const V3D &transl) {
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot) {
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &scaler) { cov_gyr_scale = scaler; }

void ImuProcess::set_acc_cov(const V3D &scaler) { cov_acc_scale = scaler; }

void ImuProcess::set_gyr_bias_cov(const V3D &b_g) { cov_bias_gyr = b_g; }

void ImuProcess::set_acc_bias_cov(const V3D &b_a) { cov_bias_acc = b_a; }

void ImuProcess::IMU_init(const MeasureGroup &meas,
                          esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                          int &N) {
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/

  V3D cur_acc, cur_gyr;

  if (b_first_frame_) {
    Reset();
    N                   = 1;
    b_first_frame_      = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    first_lidar_time = meas.lidar_beg_time;
  }

  for (const auto &imu : meas.imu) {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    const V3D previous_mean_gyr=mean_gyr;
    const V3D previous_mean_acc=mean_acc;
    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;

    if(std::getenv("AEROMIND_DECLARED_UNBIASED_IMU") && std::strcmp(std::getenv("AEROMIND_DECLARED_UNBIASED_IMU"),"1")==0){
      cov_acc=cov_acc*(N-1.0)/N+(cur_acc-previous_mean_acc).cwiseProduct(cur_acc-mean_acc)/N;
    }else {
    cov_acc = cov_acc * (N - 1.0) / N +
              (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    }
    if(std::getenv("AEROMIND_LIO_CONSISTENCY_FIX") && std::strcmp(std::getenv("AEROMIND_LIO_CONSISTENCY_FIX"),"1")==0){
      // Welford population variance: the residual after mean update alone is insufficient.
      cov_gyr=cov_gyr*(N-1.0)/N+(cur_gyr-previous_mean_gyr).cwiseProduct(cur_gyr-mean_gyr)/N;
    }else {
      cov_gyr = cov_gyr * (N - 1.0) / N +
              (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);
    }

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    N++;
  }
  state_ikfom init_state = kf_state.get_x();
  init_state.grav        = S2(-mean_acc / mean_acc.norm() * G_m_s2);
  if(std::getenv("AEROMIND_LIO_CONSISTENCY_FIX") && std::strcmp(std::getenv("AEROMIND_LIO_CONSISTENCY_FIX"),"1")==0){
    // Keep gravity length consistent with the already-used accelerometer normalization.
    init_state.grav.length=G_m_s2;
    init_state.grav.vec=-mean_acc/mean_acc.norm()*G_m_s2;
  }


  // state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  init_state.bg           = mean_gyr;
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;
  kf_state.change_x(init_state);

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
  init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
  init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
  init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
  init_P(21, 21) = init_P(22, 22) = 0.00001;
  // The local world frame is the first IMU pose, not an uncertain global pose.
  // A positive numerical anchor keeps matrix inversion nonsingular; no GT input.
  if(std::getenv("AEROMIND_LIO_CONSISTENCY_FIX") && std::strcmp(std::getenv("AEROMIND_LIO_CONSISTENCY_FIX"),"1")==0){
    for(int j=0;j<6;j++)init_P(j,j)=1e-9;
    for(int j=0;j<3;j++)init_P(15+j,15+j)=std::max(cov_gyr[j]/std::max(1,N-2),1e-12);
  }
  // Opt-in simulation prior: the launcher must verify the actual sensor model has
  // zero persistent/dynamic bias and the declared Gaussian noise. Not a real-IMU prior.
  if(std::getenv("AEROMIND_DECLARED_UNBIASED_IMU") && std::strcmp(std::getenv("AEROMIND_DECLARED_UNBIASED_IMU"),"1")==0){
    if(!(std::getenv("AEROMIND_LIO_CONSISTENCY_FIX") && std::strcmp(std::getenv("AEROMIND_LIO_CONSISTENCY_FIX"),"1")==0))throw std::runtime_error("declared IMU prior requires consistency profile");
    const double mean_den=std::max(1,N-2);
    V3D acc_mean_var;
    for(int j=0;j<3;j++){
      acc_mean_var[j]=std::max(std::max(cov_acc[j],cov_acc_scale[j])/mean_den,1e-12);
      init_P(18+j,18+j)=acc_mean_var[j];
      init_P(15+j,15+j)=std::max(std::max(cov_gyr[j],cov_gyr_scale[j])/mean_den,1e-12);
    }
    const double gravity_direction_var=std::max(2.0*acc_mean_var.maxCoeff()/mean_acc.squaredNorm(),1e-12);
    init_P(21,21)=init_P(22,22)=gravity_direction_var;
  }
  kf_state.change_P(init_P);
  last_imu_ = meas.imu.back();
}

state_ikfom ImuProcess::IntegrateIMU(const std::deque<sensor_msgs::msg::Imu> imu_queue,
                                     esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state) {
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;

  input_ikfom in;
  const auto &head = imu_queue[0];
  const auto &tail = imu_queue[1];

  angvel_avr << 0.5 * (head.angular_velocity.x + tail.angular_velocity.x),
      0.5 * (head.angular_velocity.y + tail.angular_velocity.y),
      0.5 * (head.angular_velocity.z + tail.angular_velocity.z);
  acc_avr << 0.5 * (head.linear_acceleration.x + tail.linear_acceleration.x),
      0.5 * (head.linear_acceleration.y + tail.linear_acceleration.y),
      0.5 * (head.linear_acceleration.z + tail.linear_acceleration.z);

  acc_avr = acc_avr * G_m_s2 / mean_acc.norm();  // - state_inout.ba;

  double dt = rclcpp::Time(tail.header.stamp).seconds() - rclcpp::Time(head.header.stamp).seconds();

  in.acc                         = acc_avr;
  in.gyro                        = angvel_avr;
  Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
  Q.block<3, 3>(3, 3).diagonal() = cov_acc;
  Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
  Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
  kf_state.predict(dt, Q, in);

  return kf_state.get_x();
}


state_ikfom ImuProcess::IntegrateIMUInterval(const std::deque<sensor_msgs::msg::Imu> &queue,
    int64_t start_ns, esekfom::esekf<state_ikfom,12,input_ikfom> &kf_state) {
  if(queue.size()!=2)throw std::runtime_error("IMU pair required");
  const auto &head=queue[0];const auto &tail=queue[1];
  int64_t a=rclcpp::Time(head.header.stamp).nanoseconds(),b=rclcpp::Time(tail.header.stamp).nanoseconds();
  if(b<=a || start_ns<a || start_ns>=b)throw std::runtime_error("unbracketed IMU prediction interval");
  if(!mean_acc.allFinite() || !(mean_acc.norm()>0))throw std::runtime_error("invalid IMU normalization");
  V3D ah(head.linear_acceleration.x,head.linear_acceleration.y,head.linear_acceleration.z);
  V3D at(tail.linear_acceleration.x,tail.linear_acceleration.y,tail.linear_acceleration.z);
  V3D gh(head.angular_velocity.x,head.angular_velocity.y,head.angular_velocity.z);
  V3D gt(tail.angular_velocity.x,tail.angular_velocity.y,tail.angular_velocity.z);
  if(!ah.allFinite() || !at.allFinite() || !gh.allFinite() || !gt.allFinite())throw std::runtime_error("nonfinite IMU interval");
  // Integrate only the requested subinterval. Linear interpolation is internal
  // numerical integration; no new sensor message or timestamp is manufactured.
  double alpha=static_cast<double>(start_ns-a)/static_cast<double>(b-a);
  input_ikfom in;in.acc=.5*(ah+alpha*(at-ah)+at)*G_m_s2/mean_acc.norm();
  in.gyro=.5*(gh+alpha*(gt-gh)+gt);
  Q.block<3,3>(0,0).diagonal()=cov_gyr;Q.block<3,3>(3,3).diagonal()=cov_acc;
  Q.block<3,3>(6,6).diagonal()=cov_bias_gyr;Q.block<3,3>(9,9).diagonal()=cov_bias_acc;
  double dt = static_cast<double>(b-start_ns)*1e-9;
    kf_state.predict(dt,Q,in);return kf_state.get_x();
}

void ImuProcess::UndistortPcl(const MeasureGroup &meas,
                              esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                              PointCloudXYZI &pcl_out) {
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
// AEROMIND_INPUT_BEGIN
  aeromind_input::capture("PROPAGATION_INPUT", [&](auto &e) { e.lidar_begin=meas.lidar_beg_time; e.lidar_end=meas.lidar_end_time; e.last_lidar_end=last_lidar_end_time_; aeromind_input::samples(e,v_imu); aeromind_input::filter(e,kf_state); });
// AEROMIND_INPUT_END
  const double &imu_end_time = rclcpp::Time(v_imu.back()->header.stamp).seconds();
  const double &pcl_beg_time = meas.lidar_beg_time;
  const double &pcl_end_time = meas.lidar_end_time;

  /*** sort point clouds by offset time ***/
  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", "
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

  /*** Initialize IMU pose ***/
  state_ikfom imu_state = kf_state.get_x();
  IMUpose.clear();
  IMUpose.push_back(set_pose6d(0.0,
                               acc_s_last,
                               angvel_last,
                               imu_state.vel,
                               imu_state.pos,
                               imu_state.rot.toRotationMatrix()));

  /*** forward propagation at each imu point ***/
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
  M3D R_imu;

  double dt = 0;

  input_ikfom in;
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);

    if (rclcpp::Time(tail->header.stamp).seconds() < last_lidar_end_time_) continue;

    angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
        0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
        0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
        0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
        0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    // fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " " <<
    // angvel_avr.transpose() << " " << acc_avr.transpose() << endl;

    acc_avr = acc_avr * G_m_s2 / mean_acc.norm();  // - state_inout.ba;

    if (rclcpp::Time(head->header.stamp).seconds() < last_lidar_end_time_) {
      dt = rclcpp::Time(tail->header.stamp).seconds() - last_lidar_end_time_;
      // dt = tail->header.stamp.toSec() - pcl_beg_time;
    } else {
      dt = rclcpp::Time(tail->header.stamp).seconds() - rclcpp::Time(head->header.stamp).seconds();
    }

    in.acc                         = acc_avr;
    in.gyro                        = angvel_avr;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    kf_state.predict(dt, Q, in);

    /* save the poses at each IMU measurements */
    imu_state   = kf_state.get_x();
    angvel_last = angvel_avr - imu_state.bg;
    acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);
    for (int i = 0; i < 3; i++) {
      acc_s_last[i] += imu_state.grav[i];
    }
    double &&offs_t = rclcpp::Time(tail->header.stamp).seconds() - pcl_beg_time;
    IMUpose.push_back(set_pose6d(offs_t,
                                 acc_s_last,
                                 angvel_last,
                                 imu_state.vel,
                                 imu_state.pos,
                                 imu_state.rot.toRotationMatrix()));
  }

  /*** calculated the pos and attitude prediction at the frame-end ***/
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt          = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, in);

  imu_state            = kf_state.get_x();
  last_imu_            = meas.imu.back();
  last_lidar_end_time_ = pcl_end_time;

  /*** undistort each lidar point (backward propagation) ***/
  if (pcl_out.points.begin() == pcl_out.points.end()) return;
  // IMU propagation and last-lidar bookkeeping above still execute. A truly
  // simultaneous point set needs no motion compensation at its own timestamp.
  // Do not extrapolate rays through an IMU interval boundary. Real nonzero
  // point-time scans continue through the original compensation algorithm.
  if (std::all_of(pcl_out.points.begin(), pcl_out.points.end(),
      [](const auto &point) { return point.curvature == 0.0f; })) return;

  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu << MAT_FROM_ARRAY(head->rot);
    // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
    vel_imu << VEC_FROM_ARRAY(head->vel);
    pos_imu << VEC_FROM_ARRAY(head->pos);
    acc_imu << VEC_FROM_ARRAY(tail->acc);
    angvel_avr << VEC_FROM_ARRAY(tail->gyr);

    for (; it_pcl->curvature / static_cast<double>(1000) > head->offset_time; it_pcl--) {
      dt = it_pcl->curvature / static_cast<double>(1000) - head->offset_time;

      /* Transform to the 'end' frame, using only the rotation
       * Note: Compensation direction is INVERSE of Frame's moving direction
       * So if we want to compensate a point at timestamp-i to the frame-e
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame
       */
      M3D R_i(R_imu * Exp(angvel_avr, dt));

      V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
      V3D P_compensate =
          imu_state.offset_R_L_I.conjugate() *
          (imu_state.rot.conjugate() *
               (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) -
           imu_state.offset_T_L_I);  // not accurate!

      // save Undistorted points and their rotation
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin()) return;
    }
  }
}

void ImuProcess::Process(const MeasureGroup &meas,
                         esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                         PointCloudXYZI::Ptr cur_pcl_un_) {
  if (meas.imu.empty()) {
// AEROMIND_EMPTY_DIAGNOSTIC_BEGIN
    aeromind_input::capture("PROCESS_EMPTY_BEFORE", [&](auto &e) { e.lidar_begin=meas.lidar_beg_time; e.lidar_end=meas.lidar_end_time; e.last_lidar_end=last_lidar_end_time_; e.init_count=init_iter_num; e.needs_init=imu_need_init_; e.first_init=b_first_frame_; for(const auto *v : {&mean_acc,&mean_gyr,&cov_acc,&cov_gyr,&cov_bias_gyr,&cov_bias_acc,&cov_acc_scale,&cov_gyr_scale,&angvel_last}) for(int i=0;i<3;i++) e.statistics.push_back((*v)[i]); aeromind_input::filter(e,kf_state); });
    aeromind_input::capture("PROCESS_EMPTY_AFTER", [&](auto &e) { e.lidar_begin=meas.lidar_beg_time; e.lidar_end=meas.lidar_end_time; e.last_lidar_end=last_lidar_end_time_; e.init_count=init_iter_num; e.needs_init=imu_need_init_; e.first_init=b_first_frame_; for(const auto *v : {&mean_acc,&mean_gyr,&cov_acc,&cov_gyr,&cov_bias_gyr,&cov_bias_acc,&cov_acc_scale,&cov_gyr_scale,&angvel_last}) for(int i=0;i<3;i++) e.statistics.push_back((*v)[i]); aeromind_input::filter(e,kf_state); });
// AEROMIND_EMPTY_DIAGNOSTIC_END
    return;
  }
  assert(meas.lidar != nullptr);
// AEROMIND_STARTUP_FIX_BEGIN
  const double tail_time = rclcpp::Time(meas.imu.back()->header.stamp).seconds();
  if (!std::isfinite(meas.lidar_beg_time) || !std::isfinite(meas.lidar_end_time) ||
      meas.lidar_end_time < meas.lidar_beg_time || tail_time > meas.lidar_end_time + 1e-9) {
    cur_pcl_un_->clear();
    RCLCPP_WARN(rclcpp::get_logger("imu_processing"), "Invalid scan interval; no propagation output");
    return;
  }
// AEROMIND_STARTUP_FIX_END

// AEROMIND_INPUT_BEGIN
  aeromind_input::capture("PROCESS_BEFORE", [&](auto &e) { e.lidar_begin=meas.lidar_beg_time; e.lidar_end=meas.lidar_end_time; e.init_count=init_iter_num; e.needs_init=imu_need_init_; e.first_init=b_first_frame_; aeromind_input::samples(e,meas.imu); for(const auto *v : {&mean_acc,&mean_gyr,&cov_acc,&cov_gyr,&cov_bias_gyr,&cov_bias_acc,&cov_acc_scale,&cov_gyr_scale,&angvel_last}) for(int i=0;i<3;i++) e.statistics.push_back((*v)[i]); aeromind_input::filter(e,kf_state); });
// AEROMIND_INPUT_END
  if (imu_need_init_) {
    /// The very first lidar frame
    IMU_init(meas, kf_state, init_iter_num);

    imu_need_init_ = true;

    last_imu_ = meas.imu.back();

    state_ikfom imu_state = kf_state.get_x();
    const bool consistency_fix=std::getenv("AEROMIND_LIO_CONSISTENCY_FIX") && std::strcmp(std::getenv("AEROMIND_LIO_CONSISTENCY_FIX"),"1")==0;
    const bool initialization_complete=consistency_fix ?
      (init_iter_num-1>=1250 && meas.lidar_end_time-first_lidar_time>=5.0-1e-9) : init_iter_num>MAX_INI_COUNT;
    if (initialization_complete) {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;

      cov_acc = cov_acc_scale;
      cov_gyr = cov_gyr_scale;
// AEROMIND_STARTUP_FIX_BEGIN
      // The initialized KF represents this scan's end, not epoch zero or the next scan's beginning.
      const double norm = mean_acc.norm();
      if (!std::isfinite(norm) || norm <= 1e-9) {
        imu_need_init_ = true;
        cur_pcl_un_->clear();
        return;
      }
      last_lidar_end_time_ = meas.lidar_end_time;
      const auto &sample = *last_imu_;
      const auto initialized = kf_state.get_x();
      angvel_last = V3D(sample.angular_velocity.x,sample.angular_velocity.y,sample.angular_velocity.z) - initialized.bg;
      acc_s_last = initialized.rot * (V3D(sample.linear_acceleration.x,sample.linear_acceleration.y,sample.linear_acceleration.z) * G_m_s2 / norm - initialized.ba);
      acc_s_last += V3D(initialized.grav);
      propagation_anchor_valid_ = angvel_last.allFinite() && acc_s_last.allFinite();
      if (!propagation_anchor_valid_) {
        imu_need_init_ = true;
        cur_pcl_un_->clear();
        return;
      }
// AEROMIND_STARTUP_FIX_END
// AEROMIND_INPUT_BEGIN
      aeromind_input::capture("STARTUP_ANCHOR", [&](auto &e) { e.lidar_begin=meas.lidar_beg_time; e.lidar_end=meas.lidar_end_time; e.last_lidar_end=last_lidar_end_time_; e.init_count=init_iter_num; e.samples.push_back(aeromind_input::sample(*last_imu_)); for(const auto *v : {&angvel_last,&acc_s_last}) for(int i=0;i<3;i++) e.statistics.push_back((*v)[i]); aeromind_input::filter(e,kf_state); });
// AEROMIND_INPUT_END
      RCLCPP_INFO(rclcpp::get_logger("imu_processing"), "IMU Initial Done");
      // RCLCPP_INFO(rclcpp::get_logger("imu_processing"),
      // "IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc
      // covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",
      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(),
      //          cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1],
      //          cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"), std::ios::out);
    }

// AEROMIND_INPUT_BEGIN
  aeromind_input::capture("PROCESS_AFTER_INIT", [&](auto &e) { e.lidar_begin=meas.lidar_beg_time; e.lidar_end=meas.lidar_end_time; e.init_count=init_iter_num; e.needs_init=imu_need_init_; e.first_init=b_first_frame_; aeromind_input::samples(e,meas.imu); for(const auto *v : {&mean_acc,&mean_gyr,&cov_acc,&cov_gyr,&cov_bias_gyr,&cov_bias_acc,&cov_acc_scale,&cov_gyr_scale,&angvel_last}) for(int i=0;i<3;i++) e.statistics.push_back((*v)[i]); aeromind_input::filter(e,kf_state); });
// AEROMIND_INPUT_END
    return;
  }

// AEROMIND_STARTUP_FIX_BEGIN
  if (!propagation_anchor_valid_ || meas.lidar_end_time <= last_lidar_end_time_ ||
      tail_time <= last_lidar_end_time_) {
    cur_pcl_un_->clear();
    RCLCPP_WARN(rclcpp::get_logger("imu_processing"), "No valid forward propagation interval; no output");
    return;
  }
// AEROMIND_STARTUP_FIX_END
  UndistortPcl(meas, kf_state, *cur_pcl_un_);
// AEROMIND_INPUT_BEGIN
  aeromind_input::capture("PROCESS_AFTER_PROPAGATION", [&](auto &e) { e.last_lidar_end=last_lidar_end_time_; e.lidar_begin=meas.lidar_beg_time; e.lidar_end=meas.lidar_end_time; e.init_count=init_iter_num; e.needs_init=imu_need_init_; e.first_init=b_first_frame_; aeromind_input::samples(e,meas.imu); for(const auto *v : {&mean_acc,&mean_gyr,&cov_acc,&cov_gyr,&cov_bias_gyr,&cov_bias_acc,&cov_acc_scale,&cov_gyr_scale,&angvel_last}) for(int i=0;i<3;i++) e.statistics.push_back((*v)[i]); aeromind_input::filter(e,kf_state); });
// AEROMIND_INPUT_END
}
