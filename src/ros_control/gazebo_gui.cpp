#include <cmath>
#include <sstream>
#include <string>

#include <QApplication>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

namespace
{
geometry_msgs::Quaternion quaternionFromRpy(double roll, double pitch, double yaw)
{
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  geometry_msgs::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

QDoubleSpinBox* makeSpinBox(double value, double min, double max, double step)
{
  auto* box = new QDoubleSpinBox();
  box->setRange(min, max);
  box->setSingleStep(step);
  box->setDecimals(3);
  box->setValue(value);
  return box;
}
}  // namespace

class GazeboRespawnGUI : public QWidget
{
  Q_OBJECT

public:
  explicit GazeboRespawnGUI(QWidget* parent = nullptr)
    : QWidget(parent),
      nh_(),
      pnh_("~"),
      delete_client_(nh_.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model")),
      spawn_client_(nh_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model"))
  {
    std::string model_name;
    std::string robot_description_param;
    std::string reference_frame;
    std::string robot_namespace;
    double x;
    double y;
    double z;
    double roll_deg;
    double pitch_deg;
    double yaw_deg;

    pnh_.param<std::string>("model_name", model_name, "crawler");
    pnh_.param<std::string>("robot_description_param", robot_description_param, "robot_description");
    pnh_.param<std::string>("reference_frame", reference_frame, "world");
    pnh_.param<std::string>("robot_namespace", robot_namespace, "");
    pnh_.param<double>("x", x, 0.0);
    pnh_.param<double>("y", y, 0.0);
    pnh_.param<double>("z", z, 0.6);
    pnh_.param<double>("roll_deg", roll_deg, 0.0);
    pnh_.param<double>("pitch_deg", pitch_deg, 0.0);
    pnh_.param<double>("yaw_deg", yaw_deg, 0.0);

    setWindowTitle("Gazebo Robot Respawn");

    model_name_edit_ = new QLineEdit(QString::fromStdString(model_name));
    robot_description_edit_ = new QLineEdit(QString::fromStdString(robot_description_param));
    reference_frame_edit_ = new QLineEdit(QString::fromStdString(reference_frame));
    robot_namespace_edit_ = new QLineEdit(QString::fromStdString(robot_namespace));

    x_spin_ = makeSpinBox(x, -100.0, 100.0, 0.1);
    y_spin_ = makeSpinBox(y, -100.0, 100.0, 0.1);
    z_spin_ = makeSpinBox(z, -10.0, 100.0, 0.1);
    roll_spin_ = makeSpinBox(roll_deg, -180.0, 180.0, 5.0);
    pitch_spin_ = makeSpinBox(pitch_deg, -180.0, 180.0, 5.0);
    yaw_spin_ = makeSpinBox(yaw_deg, -180.0, 180.0, 5.0);

    auto* form = new QFormLayout();
    form->addRow("Model name", model_name_edit_);
    form->addRow("Robot description param", robot_description_edit_);
    form->addRow("Reference frame", reference_frame_edit_);
    form->addRow("Robot namespace", robot_namespace_edit_);
    form->addRow("X [m]", x_spin_);
    form->addRow("Y [m]", y_spin_);
    form->addRow("Z [m]", z_spin_);
    form->addRow("Roll [deg]", roll_spin_);
    form->addRow("Pitch [deg]", pitch_spin_);
    form->addRow("Yaw [deg]", yaw_spin_);

    auto* respawn_button = new QPushButton("Delete and respawn");
    auto* delete_button = new QPushButton("Delete only");
    auto* upright_button = new QPushButton("Set upright");
    status_label_ = new QLabel("Ready");
    status_label_->setWordWrap(true);

    connect(respawn_button, &QPushButton::clicked, this, &GazeboRespawnGUI::deleteAndRespawn);
    connect(delete_button, &QPushButton::clicked, this, &GazeboRespawnGUI::deleteOnly);
    connect(upright_button, &QPushButton::clicked, this, &GazeboRespawnGUI::setUpright);

    auto* buttons = new QHBoxLayout();
    buttons->addWidget(respawn_button);
    buttons->addWidget(delete_button);
    buttons->addWidget(upright_button);

    auto* layout = new QVBoxLayout();
    layout->addLayout(form);
    layout->addLayout(buttons);
    layout->addWidget(status_label_);
    setLayout(layout);
  }

private slots:
  void deleteAndRespawn()
  {
    deleteModel(false);
    spawnModel();
  }

  void deleteOnly()
  {
    deleteModel(true);
  }

  void setUpright()
  {
    roll_spin_->setValue(0.0);
    pitch_spin_->setValue(0.0);
  }

private:
  void setStatus(const std::string& message)
  {
    status_label_->setText(QString::fromStdString(message));
  }

  std::string modelName() const
  {
    return model_name_edit_->text().toStdString();
  }

  bool deleteModel(bool report_success)
  {
    gazebo_msgs::DeleteModel srv;
    srv.request.model_name = modelName();

    if (!delete_client_.waitForExistence(ros::Duration(2.0))) {
      setStatus("Delete service is not available: /gazebo/delete_model");
      return false;
    }

    if (!delete_client_.call(srv)) {
      setStatus("Failed to call /gazebo/delete_model");
      return false;
    }

    if (!srv.response.success) {
      ROS_WARN("DeleteModel returned false for %s: %s",
               srv.request.model_name.c_str(),
               srv.response.status_message.c_str());
      if (report_success) {
        setStatus("Delete failed: " + srv.response.status_message);
      }
      return false;
    }

    if (report_success) {
      setStatus("Deleted model: " + srv.request.model_name);
    }
    return true;
  }

  void spawnModel()
  {
    const std::string description_param = robot_description_edit_->text().toStdString();
    std::string robot_xml;
    if (!nh_.getParam(description_param, robot_xml) || robot_xml.empty()) {
      setStatus("Missing or empty robot description param: " + description_param);
      return;
    }

    if (!spawn_client_.waitForExistence(ros::Duration(2.0))) {
      setStatus("Spawn service is not available: /gazebo/spawn_urdf_model");
      return;
    }

    gazebo_msgs::SpawnModel srv;
    srv.request.model_name = modelName();
    srv.request.model_xml = robot_xml;
    srv.request.robot_namespace = robot_namespace_edit_->text().toStdString();
    srv.request.reference_frame = reference_frame_edit_->text().toStdString();
    srv.request.initial_pose = currentPose();

    if (!spawn_client_.call(srv)) {
      setStatus("Failed to call /gazebo/spawn_urdf_model");
      return;
    }

    if (!srv.response.success) {
      setStatus("Spawn failed: " + srv.response.status_message);
      return;
    }

    std::ostringstream stream;
    stream << "Spawned " << srv.request.model_name << " at x=" << x_spin_->value()
           << " y=" << y_spin_->value() << " z=" << z_spin_->value();
    setStatus(stream.str());
  }

  geometry_msgs::Pose currentPose() const
  {
    geometry_msgs::Pose pose;
    pose.position.x = x_spin_->value();
    pose.position.y = y_spin_->value();
    pose.position.z = z_spin_->value();

    const double deg_to_rad = M_PI / 180.0;
    pose.orientation = quaternionFromRpy(
        roll_spin_->value() * deg_to_rad,
        pitch_spin_->value() * deg_to_rad,
        yaw_spin_->value() * deg_to_rad);
    return pose;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::ServiceClient delete_client_;
  ros::ServiceClient spawn_client_;

  QLineEdit* model_name_edit_;
  QLineEdit* robot_description_edit_;
  QLineEdit* reference_frame_edit_;
  QLineEdit* robot_namespace_edit_;
  QDoubleSpinBox* x_spin_;
  QDoubleSpinBox* y_spin_;
  QDoubleSpinBox* z_spin_;
  QDoubleSpinBox* roll_spin_;
  QDoubleSpinBox* pitch_spin_;
  QDoubleSpinBox* yaw_spin_;
  QLabel* status_label_;
};

#include "gazebo_gui.moc"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "gazebo_respawn_gui");
  QApplication app(argc, argv);

  GazeboRespawnGUI window;
  window.show();

  return app.exec();
}
