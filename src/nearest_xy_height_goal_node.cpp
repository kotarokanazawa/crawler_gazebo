#include <limits>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class NearestXYHeightGoalNode : public rclcpp::Node
{
public:
  NearestXYHeightGoalNode()
  : Node("nearest_xy_height_goal_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/move_base_simple/goal");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_map");
    output_topic_ = declare_parameter<std::string>("output_topic", "/move_base_simple/goal/se3");
    search_radius_ = declare_parameter("search_radius", 0.30);
    transform_timeout_ = declare_parameter("transform_timeout", 0.2);
    use_nan_filter_ = declare_parameter("use_nan_filter", true);

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 1, std::bind(&NearestXYHeightGoalNode::goalCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, 1, std::bind(&NearestXYHeightGoalNode::cloudCallback, this, std::placeholders::_1));
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      output_topic_, rclcpp::QoS(1).transient_local());

    RCLCPP_INFO(get_logger(), "subscribed goal: %s", goal_topic_.c_str());
    RCLCPP_INFO(get_logger(), "subscribed cloud: %s", cloud_topic_.c_str());
    RCLCPP_INFO(get_logger(), "publish se3 goal: %s", output_topic_.c_str());
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex cloud_mutex_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_{new pcl::PointCloud<pcl::PointXYZ>()};
  std::string latest_cloud_frame_;
  builtin_interfaces::msg::Time latest_cloud_stamp_;
  bool has_cloud_ = false;

  std::string goal_topic_;
  std::string cloud_topic_;
  std::string output_topic_;
  double search_radius_;
  double transform_timeout_;
  bool use_nan_filter_;

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *tmp);

    std::lock_guard<std::mutex> lock(cloud_mutex_);
    latest_cloud_ = tmp;
    latest_cloud_frame_ = msg->header.frame_id;
    latest_cloud_stamp_ = msg->header.stamp;
    has_cloud_ = true;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::string cloud_frame;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      if (!has_cloud_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "no cloud received yet.");
        return;
      }
      cloud = latest_cloud_;
      cloud_frame = latest_cloud_frame_;
    }

    geometry_msgs::msg::PoseStamped goal_in_cloud;
    try {
      goal_in_cloud = tf_buffer_.transform(
        *msg, cloud_frame, tf2::durationFromSec(transform_timeout_));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(get_logger(), "failed to transform goal from %s to %s: %s",
                  msg->header.frame_id.c_str(), cloud_frame.c_str(), ex.what());
      return;
    }

    double nearest_z = 0.0;
    double nearest_dist2 = std::numeric_limits<double>::max();
    bool found = false;

    const double qx = goal_in_cloud.pose.position.x;
    const double qy = goal_in_cloud.pose.position.y;
    const double radius2 = search_radius_ * search_radius_;

    for (const auto& pt : cloud->points) {
      if (use_nan_filter_ && (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))) {
        continue;
      }
      const double dx = pt.x - qx;
      const double dy = pt.y - qy;
      const double dist2 = dx * dx + dy * dy;
      if (dist2 < nearest_dist2 && dist2 <= radius2) {
        nearest_dist2 = dist2;
        nearest_z = pt.z;
        found = true;
      }
    }

    if (!found) {
      RCLCPP_WARN(get_logger(), "no cloud point found near goal xy=(%.3f, %.3f) in frame=%s within radius=%.3f m",
                  qx, qy, cloud_frame.c_str(), search_radius_);
      return;
    }

    geometry_msgs::msg::PoseStamped se3_goal_in_cloud = goal_in_cloud;
    se3_goal_in_cloud.header.stamp = now();
    se3_goal_in_cloud.pose.position.z = nearest_z;

    geometry_msgs::msg::PoseStamped se3_goal_out;
    try {
      if (msg->header.frame_id == cloud_frame) {
        se3_goal_out = se3_goal_in_cloud;
      } else {
        se3_goal_out = tf_buffer_.transform(
          se3_goal_in_cloud, msg->header.frame_id, tf2::durationFromSec(transform_timeout_));
      }
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(get_logger(), "failed to transform se3 goal back to %s: %s",
                  msg->header.frame_id.c_str(), ex.what());
      return;
    }

    se3_goal_out.header.stamp = now();
    goal_pub_->publish(se3_goal_out);

    RCLCPP_INFO(get_logger(), "published se3 goal: frame=%s xyz=(%.3f, %.3f, %.3f)",
                se3_goal_out.header.frame_id.c_str(),
                se3_goal_out.pose.position.x,
                se3_goal_out.pose.position.y,
                se3_goal_out.pose.position.z);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NearestXYHeightGoalNode>());
  rclcpp::shutdown();
  return 0;
}
