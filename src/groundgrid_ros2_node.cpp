#include <chrono>
#include <memory>
#include <unordered_map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/image.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv_converter.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <velodyne_pointcloud/point_types.h>

#include "groundgrid/GroundGrid.h"
#include "groundgrid/GroundSegmentation.h"

namespace groundgrid
{
class GroundGridNode : public rclcpp::Node
{
public:
  using PCLPoint = velodyne_pointcloud::PointXYZIR;

  GroundGridNode() : Node("groundgrid"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    image_transport::ImageTransport it(shared_from_this());
    grid_map_cv_img_pub_ = it.advertise("groundgrid/grid_map_cv", 1);
    terrain_im_pub_ = it.advertise("groundgrid/terrain", 1);
    grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("groundgrid/grid_map", 1);
    filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("groundgrid/segmented_cloud", 1);

    groundgrid_ = std::make_shared<GroundGrid>();
    ground_segmentation_.init(groundgrid_->mDimension, groundgrid_->mResolution);

    pos_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/localization/odometry/filtered_map", 1,
      std::bind(&GroundGridNode::odom_callback, this, std::placeholders::_1));
    points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/sensors/velodyne_points", 1,
      std::bind(&GroundGridNode::points_callback, this, std::placeholders::_1));
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto start = std::chrono::steady_clock::now();
    map_ptr_ = groundgrid_->update(msg);
    auto end = std::chrono::steady_clock::now();
    RCLCPP_DEBUG(get_logger(), "grid map update took %ld ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
  }

  void points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    auto start = std::chrono::steady_clock::now();
    static size_t time_vals = 0;
    static double avg_time = 0.0;
    static double avg_cpu_time = 0.0;
    pcl::PointCloud<PCLPoint>::Ptr cloud(new pcl::PointCloud<PCLPoint>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    geometry_msgs::msg::TransformStamped mapToBaseTransform, cloudOriginTransform;

    if (!map_ptr_)
      return;

    try {
      mapToBaseTransform = tf_buffer_.lookupTransform("map", "base_link", cloud_msg->header.stamp);
      cloudOriginTransform = tf_buffer_.lookupTransform("map", "velodyne", cloud_msg->header.stamp);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Received point cloud but transforms are not available: %s", ex.what());
      return;
    }

    geometry_msgs::msg::PointStamped origin;
    origin.header = cloud_msg->header;
    origin.header.frame_id = "velodyne";
    origin.point.x = 0.0f;
    origin.point.y = 0.0f;
    origin.point.z = 0.0f;
    tf2::doTransform(origin, origin, cloudOriginTransform);

    if (cloud_msg->header.frame_id != "map") {
      geometry_msgs::msg::TransformStamped transformStamped;
      pcl::PointCloud<PCLPoint>::Ptr transformed_cloud(new pcl::PointCloud<PCLPoint>);
      transformed_cloud->header = cloud->header;
      transformed_cloud->header.frame_id = "map";
      transformed_cloud->points.reserve(cloud->points.size());
      try {
        transformStamped = tf_buffer_.lookupTransform("map", cloud_msg->header.frame_id, cloud_msg->header.stamp);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "Failed to get map transform for point cloud transformation: %s", ex.what());
        return;
      }
      geometry_msgs::msg::PointStamped psIn;
      psIn.header = cloud_msg->header;
      psIn.header.frame_id = "map";
      for (const auto & point : cloud->points) {
        psIn.point.x = point.x;
        psIn.point.y = point.y;
        psIn.point.z = point.z;
        tf2::doTransform(psIn, psIn, transformStamped);
        PCLPoint & point_transformed = transformed_cloud->points.emplace_back(point);
        point_transformed.x = psIn.point.x;
        point_transformed.y = psIn.point.y;
        point_transformed.z = psIn.point.z;
      }
      cloud = transformed_cloud;
    }
    auto end = std::chrono::steady_clock::now();
    RCLCPP_DEBUG(get_logger(), "cloud transformation took %ld ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

    auto start2 = std::chrono::steady_clock::now();
    std::clock_t c_clock = std::clock();
    sensor_msgs::msg::PointCloud2 cloud_msg_out;
    PCLPoint origin_pclPoint;
    origin_pclPoint.x = origin.point.x;
    origin_pclPoint.y = origin.point.y;
    origin_pclPoint.z = origin.point.z;
    pcl::toROSMsg(*(ground_segmentation_.filter_cloud(cloud, origin_pclPoint, mapToBaseTransform, *map_ptr_)), cloud_msg_out);
    cloud_msg_out.header = cloud_msg->header;
    cloud_msg_out.header.frame_id = "map";
    filtered_cloud_pub_->publish(cloud_msg_out);
    end = std::chrono::steady_clock::now();
    double milliseconds = std::chrono::duration<double>(end - start2).count() * 1000.0;
    double c_millis = static_cast<double>(std::clock() - c_clock) / CLOCKS_PER_SEC * 1000.0;
    avg_time = (milliseconds + time_vals * avg_time) / (time_vals + 1);
    avg_cpu_time = (c_millis + time_vals * avg_cpu_time) / (time_vals + 1);
    ++time_vals;
    RCLCPP_INFO(get_logger(), "groundgrid took %.2fms (avg: %.2fms)", milliseconds, avg_time);
    RCLCPP_DEBUG(get_logger(), "total cpu time used: %.2fms (avg: %.2fms)", c_millis, avg_cpu_time);

    grid_map_msgs::msg::GridMap grid_map_msg;
    grid_map::GridMapRosConverter::toMessage(*map_ptr_, grid_map_msg);
    grid_map_msg.info.header.stamp = cloud_msg->header.stamp;
    grid_map_pub_->publish(grid_map_msg);

    image_transport::ImageTransport it(shared_from_this());
    for (const auto & layer : map_ptr_->getLayers()) {
      if (layer_pubs_.find(layer) == layer_pubs_.end()) {
        layer_pubs_[layer] = it.advertise("/groundgrid/grid_map_cv_" + layer, 1);
      }
      publish_grid_map_layer(layer_pubs_.at(layer), layer, cloud_msg->header.stamp);
    }
    if (terrain_im_pub_.getNumSubscribers()) {
      publish_grid_map_layer(terrain_im_pub_, "terrain", cloud_msg->header.stamp);
    }
    end = std::chrono::steady_clock::now();
    RCLCPP_DEBUG(get_logger(), "overall %ld ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
  }

  void publish_grid_map_layer(const image_transport::Publisher & pub, const std::string & layer_name,
                              const rclcpp::Time & stamp)
  {
    cv::Mat img, normalized_img, color_img, mask;
    if (pub.getNumSubscribers()) {
      if (layer_name != "terrain") {
        const auto & map = *map_ptr_;
        grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, layer_name, CV_8UC1, img);
        cv::applyColorMap(img, color_img, cv::COLORMAP_TWILIGHT);
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "8UC3", color_img).toImageMsg();
        msg->header.stamp = stamp;
        pub.publish(msg);
      } else {
        const auto & map = *map_ptr_;
        img = cv::Mat(map.getSize()(0), map.getSize()(1), CV_32FC3, cv::Scalar(0, 0, 0));
        normalized_img = cv::Mat(map.getSize()(0), map.getSize()(1), CV_32FC3, cv::Scalar(0, 0, 0));
        const grid_map::Matrix & data = map["ground"];
        const grid_map::Matrix & visited_layer = map["pointsRaw"];
        const grid_map::Matrix & gp_layer = map["groundpatch"];
        const float car_height = data(181, 181);
        const float ground_min = map["ground"].minCoeff() - car_height;
        const float ground_max = map["ground"].maxCoeff() - car_height;
        if (ground_max == ground_min)
          return;
        for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
          const grid_map::Index index(*iterator);
          const float value = data(index(0), index(1));
          const float gp = gp_layer(index(0), index(1));
          const grid_map::Index imageIndex(iterator.getUnwrappedIndex());
          const float pointssum = visited_layer.block<3, 3>(index(0) - 1, index(1) - 1).sum();
          const float pointcount = visited_layer(index(0), index(1));
          img.at<cv::Point3f>(imageIndex(0), imageIndex(1)) = cv::Point3f(value, pointssum >= 27 ? 1.0f : 0.0f, pointcount);
        }
        geometry_msgs::msg::TransformStamped baseToUtmTransform;
        try {
          baseToUtmTransform = tf_buffer_.lookupTransform("utm", "base_link", stamp);
        } catch (tf2::TransformException & ex) {
          RCLCPP_WARN(get_logger(), "%s", ex.what());
          return;
        }
        geometry_msgs::msg::PointStamped ps;
        ps.header.frame_id = "base_link";
        ps.header.stamp = stamp;
        tf2::doTransform(ps, ps, baseToUtmTransform);
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC3", img).toImageMsg();
        msg->header.frame_id = std::to_string(ps.header.stamp.sec) + "_" + std::to_string(ps.point.x) + "_" + std::to_string(ps.point.y);
        pub.publish(msg);
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pos_sub_;
  image_transport::Publisher grid_map_cv_img_pub_, terrain_im_pub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
  std::unordered_map<std::string, image_transport::Publisher> layer_pubs_;

  std::shared_ptr<GroundGrid> groundgrid_;
  std::shared_ptr<grid_map::GridMap> map_ptr_;
  GroundSegmentation ground_segmentation_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
}  // namespace groundgrid

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<groundgrid::GroundGridNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
