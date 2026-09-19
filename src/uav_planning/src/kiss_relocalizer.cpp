#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

class KissRelocalizer : public rclcpp::Node {
 public:
  KissRelocalizer() : Node("kiss_relocalizer") {
    map_path_ = declare_parameter<std::string>("map_path", "");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered_2d");
    map_topic_ = declare_parameter<std::string>("map_topic", "/prior_map");
    aligned_topic_ = declare_parameter<std::string>(
        "aligned_cloud_topic", "/relocalization/aligned_cloud");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    voxel_size_ = declare_parameter<double>("voxel_size_m", 0.15);
    min_z_ = declare_parameter<double>("obstacle_min_z", 0.10);
    max_z_ = declare_parameter<double>("obstacle_max_z", 1.20);
    max_correspondence_ = declare_parameter<double>("max_correspondence_m", 1.0);
    max_rmse_ = declare_parameter<double>("max_rmse_m", 0.35);
    max_iterations_ = declare_parameter<int>("max_iterations", 40);
    min_inliers_ = declare_parameter<int>("min_inliers", 30);
    tf_rate_ = declare_parameter<double>("tf_rate_hz", 20.0);
    publish_identity_ = declare_parameter<bool>("publish_identity_until_localized", true);
    declare_parameter<double>("initial_x", 0.0);
    declare_parameter<double>("initial_y", 0.0);
    declare_parameter<double>("initial_yaw", 0.0);
    if (map_path_.empty() || !(voxel_size_ > 0.0) || !(min_z_ < max_z_) ||
        !(max_correspondence_ > 0.0) || !(max_rmse_ > 0.0) || max_iterations_ < 1 ||
        min_inliers_ < 3 || !(tf_rate_ > 0.0)) {
      throw std::invalid_argument("invalid planar relocalization parameters");
    }
    Cloud raw_map;
    if (pcl::io::loadPCDFile<Point>(map_path_, raw_map) < 0 || raw_map.empty()) {
      throw std::runtime_error("cannot load nonempty PCD prior map: " + map_path_);
    }
    prior_map_ = preprocess(raw_map);
    if (prior_map_.size() < static_cast<std::size_t>(min_inliers_)) {
      throw std::runtime_error("planar prior map has too few points after height filtering");
    }

    rclcpp::QoS latched(1);
    latched.reliable().transient_local();
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(map_topic_, latched);
    aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(aligned_topic_, latched);
    status_pub_ = create_publisher<std_msgs::msg::String>("/relocalization/status", latched);
    auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, cloud_qos,
        std::bind(&KissRelocalizer::cloudCallback, this, std::placeholders::_1));
    service_ = create_service<std_srvs::srv::Trigger>(
        "/relocalize", std::bind(&KissRelocalizer::serviceCallback, this,
                                  std::placeholders::_1, std::placeholders::_2));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / tf_rate_),
                                  std::bind(&KissRelocalizer::publishTf, this));
    map_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publishPriorMap(); });
    publishPriorMap();
    publishStatus("WAITING_FOR_SCAN map=" + map_path_);
    RCLCPP_INFO(get_logger(),
                "Planar KISS relocalizer: source=%s, target=%s, solver=[tx,ty,yaw]",
                cloud_topic_.c_str(), map_path_.c_str());
  }

 private:
  using Point = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<Point>;

  Cloud preprocess(const Cloud &input) const {
    Cloud finite;
    finite.reserve(input.size());
    std::size_t already_planar = 0;
    for (const auto &point : input) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
        if (std::abs(point.z) < 1e-4) ++already_planar;
      }
    }
    const bool projected_input = !input.empty() && already_planar * 5 >= input.size() * 4;
    for (const auto &point : input) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
      if (!projected_input && (point.z < min_z_ || point.z > max_z_)) continue;
      finite.push_back(Point(point.x, point.y, 0.0F));
    }
    Cloud::Ptr finite_ptr(new Cloud(finite));
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel.setInputCloud(finite_ptr);
    Cloud output;
    voxel.filter(output);
    output.width = static_cast<std::uint32_t>(output.size());
    output.height = 1;
    output.is_dense = true;
    return output;
  }

  static Eigen::Matrix4d se2Matrix(double x, double y, double yaw) {
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    const double c = std::cos(yaw), s = std::sin(yaw);
    result(0, 0) = c;
    result(0, 1) = -s;
    result(1, 0) = s;
    result(1, 1) = c;
    result(0, 3) = x;
    result(1, 3) = y;
    return result;
  }

  Eigen::Matrix4d coarseGuess() const {
    return se2Matrix(get_parameter("initial_x").as_double(),
                     get_parameter("initial_y").as_double(),
                     get_parameter("initial_yaw").as_double());
  }

  bool solveSe2(const Cloud &source, Eigen::Matrix4d &transform,
                std::size_t &final_inliers, double &final_rmse) const {
    Cloud::ConstPtr target(new Cloud(prior_map_));
    pcl::KdTreeFLANN<Point> tree;
    tree.setInputCloud(target);
    const double max_sq = max_correspondence_ * max_correspondence_;
    final_inliers = 0;
    final_rmse = std::numeric_limits<double>::infinity();

    for (int iteration = 0; iteration < max_iterations_; ++iteration) {
      Cloud moved;
      pcl::transformPointCloud(source, moved, transform.cast<float>());
      std::vector<Eigen::Vector2d> from;
      std::vector<Eigen::Vector2d> to;
      std::vector<double> distances;
      from.reserve(moved.size());
      to.reserve(moved.size());
      distances.reserve(moved.size());
      for (const auto &point : moved) {
        std::vector<int> index(1);
        std::vector<float> squared_distance(1);
        if (tree.nearestKSearch(point, 1, index, squared_distance) == 1 &&
            squared_distance[0] <= max_sq) {
          const auto &match = prior_map_[index[0]];
          from.emplace_back(point.x, point.y);
          to.emplace_back(match.x, match.y);
          distances.push_back(std::sqrt(squared_distance[0]));
        }
      }
      if (from.size() < static_cast<std::size_t>(min_inliers_)) return false;

      Eigen::Vector2d from_mean = Eigen::Vector2d::Zero();
      Eigen::Vector2d to_mean = Eigen::Vector2d::Zero();
      double weight_sum = 0.0;
      for (std::size_t i = 0; i < from.size(); ++i) {
        const double weight = std::min(1.0, max_rmse_ / std::max(1e-6, distances[i]));
        from_mean += weight * from[i];
        to_mean += weight * to[i];
        weight_sum += weight;
      }
      from_mean /= weight_sum;
      to_mean /= weight_sum;
      double a = 0.0, b = 0.0, squared_error = 0.0;
      for (std::size_t i = 0; i < from.size(); ++i) {
        const double weight = std::min(1.0, max_rmse_ / std::max(1e-6, distances[i]));
        const Eigen::Vector2d p = from[i] - from_mean;
        const Eigen::Vector2d q = to[i] - to_mean;
        a += weight * (p.x() * q.x() + p.y() * q.y());
        b += weight * (p.x() * q.y() - p.y() * q.x());
        squared_error += distances[i] * distances[i];
      }
      const double delta_yaw = std::atan2(b, a);
      const Eigen::Rotation2Dd rotation(delta_yaw);
      const Eigen::Vector2d delta_translation = to_mean - rotation * from_mean;
      const Eigen::Matrix4d delta = se2Matrix(
          delta_translation.x(), delta_translation.y(), delta_yaw);
      transform = delta * transform;
      final_inliers = from.size();
      final_rmse = std::sqrt(squared_error / static_cast<double>(from.size()));
      if (delta_translation.norm() < 1e-4 && std::abs(delta_yaw) < 1e-4) break;
    }
    return final_inliers >= static_cast<std::size_t>(min_inliers_) && final_rmse <= max_rmse_;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
    if (message->header.frame_id != odom_frame_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Ignoring cloud frame '%s'; expected '%s'",
                            message->header.frame_id.c_str(), odom_frame_.c_str());
      return;
    }
    Cloud raw;
    pcl::fromROSMsg(*message, raw);
    Cloud cloud = preprocess(raw);
    if (cloud.size() < static_cast<std::size_t>(min_inliers_)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    latest_cloud_ = std::move(cloud);
    have_cloud_ = true;
  }

  void serviceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    Cloud source;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_cloud_) {
        response->success = false;
        response->message = "no planar odom-frame cloud is available";
        return;
      }
      source = latest_cloud_;
    }
    publishStatus("MATCHING_SE2");
    Eigen::Matrix4d map_T_odom = coarseGuess();
    std::size_t inliers = 0;
    double rmse = 0.0;
    if (!solveSe2(source, map_T_odom, inliers, rmse)) {
      std::ostringstream text;
      text << "SE2 match rejected: inliers=" << inliers << " rmse=" << rmse;
      response->success = false;
      response->message = text.str();
      publishStatus("FAILED " + response->message);
      return;
    }

    // Enforce the transform form explicitly; the solver never has z/roll/pitch variables.
    const double yaw = std::atan2(map_T_odom(1, 0), map_T_odom(0, 0));
    map_T_odom = se2Matrix(map_T_odom(0, 3), map_T_odom(1, 3), yaw);
    Cloud aligned;
    pcl::transformPointCloud(source, aligned, map_T_odom.cast<float>());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_T_odom_ = map_T_odom;
      localized_ = true;
    }
    sensor_msgs::msg::PointCloud2 aligned_message;
    pcl::toROSMsg(aligned, aligned_message);
    aligned_message.header.frame_id = map_frame_;
    aligned_message.header.stamp = now();
    aligned_pub_->publish(aligned_message);
    publishTf();

    std::ostringstream text;
    text.setf(std::ios::fixed);
    text.precision(6);
    text << "T_map_odom x=" << map_T_odom(0, 3) << " y=" << map_T_odom(1, 3)
         << " yaw=" << yaw << " inliers=" << inliers << " rmse=" << rmse;
    response->success = true;
    response->message = text.str();
    publishStatus("LOCALIZED_SE2 " + response->message);
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void publishPriorMap() {
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(prior_map_, message);
    message.header.frame_id = map_frame_;
    message.header.stamp = now();
    map_pub_->publish(message);
  }

  void publishStatus(const std::string &value) {
    std_msgs::msg::String message;
    message.data = value;
    status_pub_->publish(message);
  }

  void publishTf() {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    bool should_publish = publish_identity_;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (localized_) {
        transform = map_T_odom_;
        should_publish = true;
      }
    }
    if (!should_publish) return;
    const double yaw = std::atan2(transform(1, 0), transform(0, 0));
    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = now();
    message.header.frame_id = map_frame_;
    message.child_frame_id = odom_frame_;
    message.transform.translation.x = transform(0, 3);
    message.transform.translation.y = transform(1, 3);
    message.transform.translation.z = 0.0;
    message.transform.rotation.z = std::sin(0.5 * yaw);
    message.transform.rotation.w = std::cos(0.5 * yaw);
    tf_broadcaster_->sendTransform(message);
  }

  std::string map_path_, cloud_topic_, map_topic_, aligned_topic_, map_frame_, odom_frame_;
  double voxel_size_, min_z_, max_z_, max_correspondence_, max_rmse_, tf_rate_;
  int max_iterations_, min_inliers_;
  bool publish_identity_;
  Cloud prior_map_, latest_cloud_;
  bool have_cloud_{false}, localized_{false};
  Eigen::Matrix4d map_T_odom_{Eigen::Matrix4d::Identity()};
  std::mutex mutex_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_, aligned_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_, map_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KissRelocalizer>());
  rclcpp::shutdown();
  return 0;
}
