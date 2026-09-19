#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PlanarCloudProjector : public rclcpp::Node {
 public:
  PlanarCloudProjector() : Node("planar_cloud_projector") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/cloud_registered");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cloud_registered_2d");
    min_z_ = declare_parameter<double>("obstacle_min_z", 0.10);
    max_z_ = declare_parameter<double>("obstacle_max_z", 1.20);
    voxel_size_ = declare_parameter<double>("planar_voxel_size", 0.10);
    if (!(min_z_ < max_z_) || !(voxel_size_ > 0.0)) {
      throw std::invalid_argument("require obstacle_min_z < obstacle_max_z and positive voxel size");
    }
    auto qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, qos, std::bind(&PlanarCloudProjector::callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Planar cloud: %s -> %s; z ROI=[%.3f, %.3f], voxel=%.3f",
                input_topic_.c_str(), output_topic_.c_str(), min_z_, max_z_, voxel_size_);
  }

 private:
  using Point = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<Point>;

  void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
    Cloud input;
    pcl::fromROSMsg(*message, input);
    Cloud::Ptr projected(new Cloud);
    projected->reserve(input.size());
    for (const auto &point : input) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
          point.z < min_z_ || point.z > max_z_) {
        continue;
      }
      projected->push_back(Point(point.x, point.y, 0.0F));
    }
    pcl::VoxelGrid<Point> voxel;
    voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel.setInputCloud(projected);
    Cloud output;
    voxel.filter(output);
    output.width = static_cast<std::uint32_t>(output.size());
    output.height = 1;
    output.is_dense = true;
    sensor_msgs::msg::PointCloud2 result;
    pcl::toROSMsg(output, result);
    result.header = message->header;
    publisher_->publish(result);
  }

  std::string input_topic_;
  std::string output_topic_;
  double min_z_;
  double max_z_;
  double voxel_size_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanarCloudProjector>());
  rclcpp::shutdown();
  return 0;
}
