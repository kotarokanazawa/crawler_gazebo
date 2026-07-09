#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <ignition/msgs/double.pb.h>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

namespace
{
double clamp(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(value, upper));
}
}  // namespace

class CrawlerIgnitionControlBridge : public rclcpp::Node
{
public:
  CrawlerIgnitionControlBridge()
  : Node("crawler_ignition_control_bridge")
  {
    this->declare_parameter<std::string>("model_name", "crawler");
    this->declare_parameter<std::string>("cmd_vel_topic", "/target/cmd_vel");
    this->declare_parameter<std::string>("joint_state_topic", "/target/joint_states");
    this->declare_parameter<double>("track_width", 0.45);
    this->declare_parameter<double>("sprocket_radius", 0.082);
    this->declare_parameter<double>("command_scale", 0.55);
    this->declare_parameter<double>("motor_max_rpm", 78.378);
    this->declare_parameter<double>("gear_ratio", 2.5556);
    this->declare_parameter<double>("max_linear_velocity", -1.0);
    this->declare_parameter<double>("max_angular_velocity", -1.0);
    this->declare_parameter<double>("command_timeout", 0.5);

    model_name_ = this->get_parameter("model_name").as_string();
    track_width_ = this->get_parameter("track_width").as_double();
    sprocket_radius_ = this->get_parameter("sprocket_radius").as_double();
    command_scale_ = this->get_parameter("command_scale").as_double();
    gear_ratio_ = this->get_parameter("gear_ratio").as_double();
    command_timeout_ = this->get_parameter("command_timeout").as_double();
    max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();

    if (track_width_ <= 0.0) {
      track_width_ = 0.45;
    }
    if (sprocket_radius_ <= 0.0) {
      sprocket_radius_ = 0.082;
    }
    if (gear_ratio_ <= 0.0) {
      gear_ratio_ = 1.0;
    }

    const double motor_max_rpm = this->get_parameter("motor_max_rpm").as_double();
    const double motor_rad_per_sec = motor_max_rpm / gear_ratio_ * 2.0 * M_PI / 60.0;
    if (max_linear_velocity_ <= 0.0) {
      max_linear_velocity_ = motor_rad_per_sec * sprocket_radius_;
    }
    if (max_angular_velocity_ <= 0.0) {
      max_angular_velocity_ = max_linear_velocity_ / (track_width_ / 2.0);
    }

    for (const auto &joint : left_velocity_joints_) {
      left_velocity_pubs_.push_back(advertiseJointCommand(joint, "cmd_vel"));
    }
    for (const auto &joint : right_velocity_joints_) {
      right_velocity_pubs_.push_back(advertiseJointCommand(joint, "cmd_vel"));
    }
    for (const auto &joint : flipper_position_joints_) {
      position_pubs_[joint] = advertiseJointCommand(joint, "0/cmd_pos");
    }

    addCmdVelSubscription(this->get_parameter("cmd_vel_topic").as_string());
    addCmdVelSubscription("/cmd_vel");
    addJointStateSubscription(this->get_parameter("joint_state_topic").as_string());
    addJointStateSubscription("/joint_states");

    for (const auto &joint : flipper_position_joints_) {
      flipper_targets_[joint] = 0.0;
    }

    timer_ = this->create_wall_timer(20ms, [this]() { publishVelocityCommands(); });

    RCLCPP_INFO(
      this->get_logger(), "Forwarding crawler commands to Ignition model [%s]",
      model_name_.c_str());
  }

private:
  ignition::transport::Node::Publisher advertiseJointCommand(
    const std::string &joint_name, const std::string &command)
  {
    const std::string topic = "/model/" + model_name_ + "/joint/" + joint_name + "/" + command;
    return ign_node_.Advertise<ignition::msgs::Double>(topic);
  }

  void addCmdVelSubscription(const std::string &topic)
  {
    if (topic.empty()) {
      return;
    }
    for (const auto &existing_topic : cmd_vel_topics_) {
      if (existing_topic == topic) {
        return;
      }
    }
    cmd_vel_topics_.push_back(topic);
    RCLCPP_INFO(this->get_logger(), "Subscribing to Twist commands on [%s]", topic.c_str());
    cmd_vel_subs_.push_back(
      this->create_subscription<geometry_msgs::msg::Twist>(
        topic, 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          const double linear = clamp(msg->linear.x, -max_linear_velocity_, max_linear_velocity_);
          const double angular = clamp(msg->angular.z, -max_angular_velocity_, max_angular_velocity_);
          const double left_linear = linear - track_width_ * angular / 2.0;
          const double right_linear = linear + track_width_ * angular / 2.0;
          std::lock_guard<std::mutex> lock(command_mutex_);
          left_velocity_ = left_linear / sprocket_radius_ * command_scale_;
          right_velocity_ = right_linear / sprocket_radius_ * command_scale_;
          last_cmd_time_ = this->now();
          received_command_ = true;
        }));
  }

  void addJointStateSubscription(const std::string &topic)
  {
    if (topic.empty()) {
      return;
    }
    for (const auto &existing_topic : joint_state_topics_) {
      if (existing_topic == topic) {
        return;
      }
    }
    joint_state_topics_.push_back(topic);
    RCLCPP_INFO(this->get_logger(), "Subscribing to flipper joint targets on [%s]", topic.c_str());
    joint_state_subs_.push_back(
      this->create_subscription<sensor_msgs::msg::JointState>(
        topic, 10,
        [this, topic](const sensor_msgs::msg::JointState::SharedPtr msg) {
          const auto count = std::min(msg->name.size(), msg->position.size());
          bool accepted = false;
          std::lock_guard<std::mutex> lock(command_mutex_);
          for (size_t i = 0; i < count; ++i) {
            if (position_pubs_.find(msg->name[i]) == position_pubs_.end()) {
              continue;
            }
            flipper_targets_[msg->name[i]] = msg->position[i];
            accepted = true;
          }
          if (accepted) {
            RCLCPP_INFO_THROTTLE(
              this->get_logger(), *this->get_clock(), 1000,
              "Received flipper joint target from [%s]", topic.c_str());
          }
        }));
  }

  void publishVelocityCommands()
  {
    double left = 0.0;
    double right = 0.0;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (received_command_ && (this->now() - last_cmd_time_).seconds() <= command_timeout_) {
        left = left_velocity_;
        right = right_velocity_;
      }
    }

    ignition::msgs::Double left_msg;
    left_msg.set_data(left);
    for (auto &pub : left_velocity_pubs_) {
      pub.Publish(left_msg);
    }

    ignition::msgs::Double right_msg;
    right_msg.set_data(right);
    for (auto &pub : right_velocity_pubs_) {
      pub.Publish(right_msg);
    }

    std::map<std::string, double> flipper_targets;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      flipper_targets = flipper_targets_;
    }
    for (const auto &[joint, position] : flipper_targets) {
      auto pub_it = position_pubs_.find(joint);
      if (pub_it == position_pubs_.end()) {
        continue;
      }
      ignition::msgs::Double cmd;
      cmd.set_data(position);
      pub_it->second.Publish(cmd);
    }
  }

  const std::vector<std::string> left_velocity_joints_{
    "sprocket_axle_left",
    "flipper_sprocket_axle_left_front",
    "flipper_sprocket_axle_left_rear",
  };
  const std::vector<std::string> right_velocity_joints_{
    "sprocket_axle_right",
    "flipper_sprocket_axle_right_front",
    "flipper_sprocket_axle_right_rear",
  };
  const std::vector<std::string> flipper_position_joints_{
    "joint_left_front",
    "joint_left_rear",
    "joint_right_front",
    "joint_right_rear",
  };

  std::string model_name_;
  double track_width_{0.45};
  double sprocket_radius_{0.082};
  double command_scale_{0.55};
  double gear_ratio_{2.5556};
  double max_linear_velocity_{-1.0};
  double max_angular_velocity_{-1.0};
  double command_timeout_{0.5};

  std::mutex command_mutex_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  bool received_command_{false};
  double left_velocity_{0.0};
  double right_velocity_{0.0};
  std::map<std::string, double> flipper_targets_;

  ignition::transport::Node ign_node_;
  std::vector<ignition::transport::Node::Publisher> left_velocity_pubs_;
  std::vector<ignition::transport::Node::Publisher> right_velocity_pubs_;
  std::map<std::string, ignition::transport::Node::Publisher> position_pubs_;

  std::vector<std::string> cmd_vel_topics_;
  std::vector<std::string> joint_state_topics_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> cmd_vel_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> joint_state_subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CrawlerIgnitionControlBridge>());
  rclcpp::shutdown();
  return 0;
}
