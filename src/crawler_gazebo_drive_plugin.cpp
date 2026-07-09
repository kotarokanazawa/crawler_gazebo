#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gazebo
{
namespace
{
double rpmToRadPerSec(const double rpm)
{
  return rpm * 2.0 * M_PI / 60.0;
}

double clamp(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(value, upper));
}

std::string sdfString(const sdf::ElementPtr &sdf, const std::string &name, const std::string &fallback)
{
  return sdf->HasElement(name) ? sdf->Get<std::string>(name) : fallback;
}

double sdfDouble(const sdf::ElementPtr &sdf, const std::string &name, const double fallback)
{
  return sdf->HasElement(name) ? sdf->Get<double>(name) : fallback;
}
}  // namespace

class CrawlerGazeboDrivePlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    cmd_vel_topic_ = sdfString(sdf, "cmd_vel_topic", "/target/cmd_vel");
    track_width_ = sdfDouble(sdf, "track_width", 0.45);
    sprocket_radius_ = sdfDouble(sdf, "sprocket_radius", 0.082);
    command_scale_ = sdfDouble(sdf, "command_scale", 0.55);
    motor_max_rpm_ = sdfDouble(sdf, "motor_max_rpm", 78.378);
    gear_ratio_ = sdfDouble(sdf, "gear_ratio", 2.5556);
    max_linear_velocity_ = sdfDouble(sdf, "max_linear_velocity", -1.0);
    max_angular_velocity_ = sdfDouble(sdf, "max_angular_velocity", -1.0);
    command_timeout_ = sdfDouble(sdf, "command_timeout", 0.5);

    if (track_width_ <= 0.0) {
      track_width_ = 0.45;
    }
    if (sprocket_radius_ <= 0.0) {
      sprocket_radius_ = 0.082;
    }
    if (gear_ratio_ <= 0.0) {
      gear_ratio_ = 1.0;
    }

    const double motor_rad_per_sec = rpmToRadPerSec(motor_max_rpm_ / gear_ratio_);
    if (max_linear_velocity_ <= 0.0) {
      max_linear_velocity_ = motor_rad_per_sec * sprocket_radius_;
    }
    if (max_angular_velocity_ <= 0.0) {
      max_angular_velocity_ = max_linear_velocity_ / (track_width_ / 2.0);
    }

    addJoint(left_joints_, "sprocket_axle_left");
    addJoint(left_joints_, "flipper_sprocket_axle_left_front");
    addJoint(left_joints_, "flipper_sprocket_axle_left_rear");
    addJoint(right_joints_, "sprocket_axle_right");
    addJoint(right_joints_, "flipper_sprocket_axle_right_front");
    addJoint(right_joints_, "flipper_sprocket_axle_right_rear");

    if (!rclcpp::ok()) {
      int argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);
      owns_rclcpp_context_ = true;
    }

    node_ = std::make_shared<rclcpp::Node>(model_->GetName() + "_gazebo_drive");
    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      std::bind(&CrawlerGazeboDrivePlugin::cmdVelCallback, this, std::placeholders::_1));
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&CrawlerGazeboDrivePlugin::onUpdate, this, std::placeholders::_1));

    gzmsg << "[CrawlerGazeboDrivePlugin] Listening on [" << cmd_vel_topic_
          << "] for model [" << model_->GetName() << "]\n";
  }

  ~CrawlerGazeboDrivePlugin() override
  {
    update_connection_.reset();
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    if (node_ && executor_) {
      executor_->remove_node(node_);
      node_.reset();
    }
    executor_.reset();
    if (owns_rclcpp_context_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

private:
  void addJoint(std::vector<physics::JointPtr> &joints, const std::string &name)
  {
    const physics::JointPtr joint = model_->GetJoint(name);
    if (!joint) {
      gzwarn << "[CrawlerGazeboDrivePlugin] Joint [" << name << "] was not found.\n";
      return;
    }
    joints.push_back(joint);
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double linear = clamp(msg->linear.x, -max_linear_velocity_, max_linear_velocity_);
    const double angular = clamp(msg->angular.z, -max_angular_velocity_, max_angular_velocity_);
    const double left_linear = linear - (track_width_ * angular / 2.0);
    const double right_linear = linear + (track_width_ * angular / 2.0);

    std::lock_guard<std::mutex> lock(command_mutex_);
    left_velocity_ = left_linear / sprocket_radius_ * command_scale_;
    right_velocity_ = right_linear / sprocket_radius_ * command_scale_;
    last_command_time_ = model_->GetWorld()->SimTime();
    received_command_ = true;
  }

  void onUpdate(const common::UpdateInfo &info)
  {
    double left = 0.0;
    double right = 0.0;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (received_command_ && (info.simTime - last_command_time_).Double() <= command_timeout_) {
        left = left_velocity_;
        right = right_velocity_;
      }
    }

    applyVelocity(left_joints_, left);
    applyVelocity(right_joints_, right);
  }

  void applyVelocity(const std::vector<physics::JointPtr> &joints, const double velocity)
  {
    for (const physics::JointPtr &joint : joints) {
      if (!joint) {
        continue;
      }
      const double effort_limit = joint->GetEffortLimit(0);
      joint->SetParam("fmax", 0,
        effort_limit > 0.0 ? effort_limit : std::numeric_limits<double>::max());
      joint->SetParam("vel", 0, velocity);
    }
  }

  physics::ModelPtr model_;
  std::vector<physics::JointPtr> left_joints_;
  std::vector<physics::JointPtr> right_joints_;
  event::ConnectionPtr update_connection_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  bool owns_rclcpp_context_{false};

  std::mutex command_mutex_;
  common::Time last_command_time_{0, 0};
  bool received_command_{false};
  double left_velocity_{0.0};
  double right_velocity_{0.0};

  std::string cmd_vel_topic_;
  double track_width_{0.45};
  double sprocket_radius_{0.082};
  double command_scale_{0.55};
  double motor_max_rpm_{78.378};
  double gear_ratio_{2.5556};
  double max_linear_velocity_{-1.0};
  double max_angular_velocity_{-1.0};
  double command_timeout_{0.5};
};

GZ_REGISTER_MODEL_PLUGIN(CrawlerGazeboDrivePlugin)
}  // namespace gazebo
