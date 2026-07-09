#include <algorithm>
#include <string>
#include <vector>

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_msgs/ContactState.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace
{
geometry_msgs::Vector3 toRosVector(const gazebo::msgs::Vector3d& v)
{
  geometry_msgs::Vector3 out;
  out.x = v.x();
  out.y = v.y();
  out.z = v.z();
  return out;
}

geometry_msgs::Wrench toRosWrench(const gazebo::msgs::JointWrench& wrench)
{
  geometry_msgs::Wrench out;
  if (wrench.has_body_1_wrench())
  {
    out.force = toRosVector(wrench.body_1_wrench().force());
    out.torque = toRosVector(wrench.body_1_wrench().torque());
  }
  return out;
}

bool containsModel(const gazebo::msgs::Contact& contact, const std::string& model_name)
{
  if (model_name.empty())
  {
    return true;
  }

  const std::string prefix = model_name + "::";
  return contact.collision1().find(prefix) != std::string::npos ||
         contact.collision2().find(prefix) != std::string::npos;
}
}  // namespace

class ContactPointsPublisher
{
public:
  explicit ContactPointsPublisher(ros::NodeHandle& nh)
    : nh_(nh),
      pnh_("~"),
      tf_buffer_(),
      tf_listener_(tf_buffer_)
  {
    pnh_.param<std::string>("world_name", world_name_, "default");
    pnh_.param<std::string>("gazebo_contacts_topic", gazebo_contacts_topic_, "~/physics/contacts");
    pnh_.param<std::string>("robot_model_name", robot_model_name_, "crawler");
    pnh_.param<std::string>("contact_source_frame", contact_source_frame_, "map");
    pnh_.param<std::string>("target_frame", target_frame_, "base_link");
    pnh_.param<std::string>("fallback_target_frame", fallback_target_frame_, "remote_robot/base_link");
    pnh_.param<double>("publish_rate", publish_rate_, 30.0);
    pnh_.param<double>("marker_scale", marker_scale_, 0.04);
    pnh_.param<bool>("publish_empty", publish_empty_, true);

    contacts_pub_ = nh_.advertise<gazebo_msgs::ContactsState>("/gazebo/contact_states", 1);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/gazebo/contact_pointcloud", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/contact_points", 1);

    gazebo_node_.reset(new gazebo::transport::Node());
    gazebo_node_->Init(world_name_);
    contact_sub_ = gazebo_node_->Subscribe(gazebo_contacts_topic_, &ContactPointsPublisher::contactsCallback, this);

    ROS_INFO_STREAM("Publishing Gazebo contacts from " << gazebo_contacts_topic_
                    << " in frame " << target_frame_
                    << " for model filter '" << robot_model_name_ << "'.");
  }

private:
  void contactsCallback(ConstContactsPtr& msg)
  {
    const ros::Time now = ros::Time::now();
    if (publish_rate_ > 0.0 && !last_publish_time_.isZero() &&
        (now - last_publish_time_).toSec() < 1.0 / publish_rate_)
    {
      return;
    }
    last_publish_time_ = now;

    geometry_msgs::TransformStamped transform;
    std::string output_frame;
    if (!lookupBaseTransform(now, transform, output_frame))
    {
      return;
    }

    gazebo_msgs::ContactsState contacts_msg;
    contacts_msg.header.stamp = now;
    contacts_msg.header.frame_id = output_frame;

    std::vector<geometry_msgs::Point> points;
    points.reserve(64);

    for (int i = 0; i < msg->contact_size(); ++i)
    {
      const gazebo::msgs::Contact& contact = msg->contact(i);
      if (!containsModel(contact, robot_model_name_))
      {
        continue;
      }

      gazebo_msgs::ContactState state;
      state.info = contact.DebugString();
      state.collision1_name = contact.collision1();
      state.collision2_name = contact.collision2();

      for (int j = 0; j < contact.position_size(); ++j)
      {
        geometry_msgs::PointStamped point_world;
        point_world.header.stamp = now;
        point_world.header.frame_id = contact_source_frame_;
        point_world.point.x = contact.position(j).x();
        point_world.point.y = contact.position(j).y();
        point_world.point.z = contact.position(j).z();

        geometry_msgs::PointStamped point_base;
        tf2::doTransform(point_world, point_base, transform);

        geometry_msgs::Vector3 contact_position;
        contact_position.x = point_base.point.x;
        contact_position.y = point_base.point.y;
        contact_position.z = point_base.point.z;
        state.contact_positions.push_back(contact_position);

        geometry_msgs::Point marker_point;
        marker_point.x = point_base.point.x;
        marker_point.y = point_base.point.y;
        marker_point.z = point_base.point.z;
        points.push_back(marker_point);
      }

      for (int j = 0; j < contact.normal_size(); ++j)
      {
        geometry_msgs::Vector3Stamped normal_world;
        normal_world.header.stamp = now;
        normal_world.header.frame_id = contact_source_frame_;
        normal_world.vector = toRosVector(contact.normal(j));

        geometry_msgs::Vector3Stamped normal_base;
        tf2::doTransform(normal_world, normal_base, transform);
        state.contact_normals.push_back(normal_base.vector);
      }

      for (int j = 0; j < contact.depth_size(); ++j)
      {
        state.depths.push_back(contact.depth(j));
      }

      for (int j = 0; j < contact.wrench_size(); ++j)
      {
        state.wrenches.push_back(toRosWrench(contact.wrench(j)));
      }

      contacts_msg.states.push_back(state);
    }

    if (!publish_empty_ && points.empty())
    {
      return;
    }

    contacts_pub_.publish(contacts_msg);
    publishPointCloud(points, now, output_frame);
    publishMarkers(points, now, output_frame);
  }

  bool lookupBaseTransform(const ros::Time& stamp,
                           geometry_msgs::TransformStamped& transform,
                           std::string& output_frame)
  {
    const std::vector<std::string> frames = { target_frame_, fallback_target_frame_ };

    for (const std::string& frame : frames)
    {
      if (frame.empty())
      {
        continue;
      }

      try
      {
        transform = tf_buffer_.lookupTransform(frame, contact_source_frame_, ros::Time(0), ros::Duration(0.02));
        output_frame = frame;
        return true;
      }
      catch (const tf2::TransformException& ex)
      {
        last_tf_error_ = ex.what();
      }
    }

    ROS_WARN_THROTTLE(2.0, "Cannot transform Gazebo contact points to base link: %s",
                      last_tf_error_.c_str());
    return false;
  }

  void publishPointCloud(const std::vector<geometry_msgs::Point>& points,
                         const ros::Time& stamp,
                         const std::string& frame_id)
  {
    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = frame_id;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (const auto& point : points)
    {
      *iter_x = static_cast<float>(point.x);
      *iter_y = static_cast<float>(point.y);
      *iter_z = static_cast<float>(point.z);
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    cloud_pub_.publish(cloud);
  }

  void publishMarkers(const std::vector<geometry_msgs::Point>& points,
                      const ros::Time& stamp,
                      const std::string& frame_id)
  {
    visualization_msgs::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = frame_id;
    marker.ns = "gazebo_contact_points";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker_scale_;
    marker.scale.y = marker_scale_;
    marker.scale.z = marker_scale_;
    marker.color.r = 1.0;
    marker.color.g = 0.12;
    marker.color.b = 0.02;
    marker.color.a = 1.0;
    marker.points = points;

    visualization_msgs::MarkerArray array;
    array.markers.push_back(marker);
    marker_pub_.publish(array);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  gazebo::transport::NodePtr gazebo_node_;
  gazebo::transport::SubscriberPtr contact_sub_;
  ros::Publisher contacts_pub_;
  ros::Publisher cloud_pub_;
  ros::Publisher marker_pub_;
  ros::Time last_publish_time_;
  std::string world_name_;
  std::string gazebo_contacts_topic_;
  std::string robot_model_name_;
  std::string contact_source_frame_;
  std::string target_frame_;
  std::string fallback_target_frame_;
  std::string last_tf_error_;
  double publish_rate_;
  double marker_scale_;
  bool publish_empty_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gazebo_contact_points_publisher");
  gazebo::client::setup(argc, argv);

  ros::NodeHandle nh;
  ContactPointsPublisher publisher(nh);
  ros::spin();

  gazebo::client::shutdown();
  return 0;
}
