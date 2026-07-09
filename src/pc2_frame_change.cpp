#include <cmath>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class FilteredPointCloudPublisher : public rclcpp::Node
{
public:
  FilteredPointCloudPublisher() : Node("filtered_pointcloud_publisher")
  {
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "remote_robot/base_frame");
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/gazebo/cloud_map", 1,
      std::bind(&FilteredPointCloudPublisher::cloudCallback, this, std::placeholders::_1));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal_dynamic", 1,
      std::bind(&FilteredPointCloudPublisher::poseCallback, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_cloud", 1);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&FilteredPointCloudPublisher::timerCallback, this));
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_cloud_ = cloud_msg;
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_pose_ = pose_msg;
    auto pose_stamped = *pose_msg;

    double roll;
    double pitch;
    double yaw;
    tf2::Quaternion q_orig(
      pose_stamped.pose.orientation.x,
      pose_stamped.pose.orientation.y,
      pose_stamped.pose.orientation.z,
      pose_stamped.pose.orientation.w);
    tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);

    tf2::Quaternion q_yaw;
    q_yaw.setRPY(0, 0, yaw);
    pose_stamped.header.stamp = now();
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose.orientation = tf2::toMsg(q_yaw);
    current_robot_basefootprint_pose_ = pose_stamped;
  }

  void poseStampedToTransformStamped(
    const geometry_msgs::msg::PoseStamped& pose,
    geometry_msgs::msg::TransformStamped& transform) const
  {
    tf2::Quaternion q(
      pose.pose.orientation.x,
      pose.pose.orientation.y,
      pose.pose.orientation.z,
      pose.pose.orientation.w);

    const tf2::Quaternion q_inv = q.inverse();
    const tf2::Vector3 translation(
      pose.pose.position.x,
      pose.pose.position.y,
      pose.pose.position.z);
    const tf2::Vector3 translation_inv = -tf2::quatRotate(q_inv, translation);

    transform.transform.translation.x = translation_inv.x();
    transform.transform.translation.y = translation_inv.y();
    transform.transform.translation.z = translation_inv.z();
    transform.transform.rotation.x = q_inv.x();
    transform.transform.rotation.y = q_inv.y();
    transform.transform.rotation.z = q_inv.z();
    transform.transform.rotation.w = q_inv.w();
  }

  void pointcloudGlobalToLocal(
    const sensor_msgs::msg::PointCloud2& msg,
    const geometry_msgs::msg::TransformStamped& transform_stamped)
  {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(msg, pcl_cloud);

    tf2::Quaternion q(
      transform_stamped.transform.rotation.x,
      transform_stamped.transform.rotation.y,
      transform_stamped.transform.rotation.z,
      transform_stamped.transform.rotation.w);

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << transform_stamped.transform.translation.x,
      transform_stamped.transform.translation.y,
      transform_stamped.transform.translation.z;
    Eigen::Quaternionf quaternion(q.w(), q.x(), q.y(), q.z());
    transform.rotate(quaternion);

    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(pcl_cloud, transformed_cloud, transform);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& pt : transformed_cloud.points) {
      if (std::hypot(pt.x, pt.y) <= 1.0) {
        filtered->points.push_back(pt);
      }
    }

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(filtered);
    voxel.setLeafSize(0.02f, 0.02f, 0.02f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    voxel.filter(*downsampled);

    downsampled->width = downsampled->points.size();
    downsampled->height = 1;
    downsampled->is_dense = true;

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*downsampled, out_msg);
    out_msg.header.frame_id = robot_base_frame_;
    out_msg.header.stamp = now();
    pub_->publish(out_msg);
  }

  void timerCallback()
  {
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    geometry_msgs::msg::PoseStamped pose;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!latest_cloud_ || !latest_pose_) {
        return;
      }
      cloud = latest_cloud_;
      pose = current_robot_basefootprint_pose_;
    }

    if (!cloud->data.empty()) {
      geometry_msgs::msg::TransformStamped map_to_basefootprint_transform;
      poseStampedToTransformStamped(pose, map_to_basefootprint_transform);
      pointcloudGlobalToLocal(*cloud, map_to_basefootprint_transform);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  geometry_msgs::msg::PoseStamped::SharedPtr latest_pose_;
  std::mutex data_mutex_;
  geometry_msgs::msg::PoseStamped current_robot_basefootprint_pose_;
  std::string robot_base_frame_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FilteredPointCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}
