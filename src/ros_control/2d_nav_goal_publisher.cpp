#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

#include <QApplication>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QDoubleValidator>

class NavGoalWidget : public QWidget
{
public:
    explicit NavGoalWidget(ros::NodeHandle& nh, QWidget* parent = nullptr)
        : QWidget(parent)
    {
        // ROS publisher
        pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

        // --- GUI layout ---
        auto* mainLayout = new QVBoxLayout(this);

        // x
        {
            auto* layout = new QHBoxLayout();
            auto* label = new QLabel("x [m]:");
            edit_x_ = new QLineEdit("0.0");
            edit_x_->setValidator(new QDoubleValidator(this));
            layout->addWidget(label);
            layout->addWidget(edit_x_);
            mainLayout->addLayout(layout);
        }

        // y
        {
            auto* layout = new QHBoxLayout();
            auto* label = new QLabel("y [m]:");
            edit_y_ = new QLineEdit("0.0");
            edit_y_->setValidator(new QDoubleValidator(this));
            layout->addWidget(label);
            layout->addWidget(edit_y_);
            mainLayout->addLayout(layout);
        }

        // yaw (deg)
        {
            auto* layout = new QHBoxLayout();
            auto* label = new QLabel("yaw [deg]:");
            edit_yaw_deg_ = new QLineEdit("0.0");
            edit_yaw_deg_->setValidator(new QDoubleValidator(this));
            layout->addWidget(label);
            layout->addWidget(edit_yaw_deg_);
            mainLayout->addLayout(layout);
        }

        // send button
        auto* sendButton = new QPushButton("Send 2D Nav Goal");
        mainLayout->addWidget(sendButton);

        // ボタンが押されたときに publish
        connect(sendButton, &QPushButton::clicked, [this]() {
            sendGoal();
        });

        setWindowTitle("2D Nav Goal Sender");
        resize(300, 150);
    }

private:
    void sendGoal()
    {
        bool ok_x, ok_y, ok_yaw;
        double x   = edit_x_->text().toDouble(&ok_x);
        double y   = edit_y_->text().toDouble(&ok_y);
        double yaw_deg = edit_yaw_deg_->text().toDouble(&ok_yaw);

        if (!ok_x || !ok_y || !ok_yaw) {
            ROS_WARN("Invalid input. Please enter numeric values.");
            return;
        }

        double yaw_rad = yaw_deg * M_PI / 180.0;

        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";   // 必要に応じて "odom" などに変更

        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = 0.0;

        goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_rad);

        pub_.publish(goal);
        ROS_INFO("Published 2D Nav Goal: x=%.3f, y=%.3f, yaw=%.3f[deg]", x, y, yaw_deg);
    }

    ros::Publisher pub_;
    QLineEdit* edit_x_;
    QLineEdit* edit_y_;
    QLineEdit* edit_yaw_deg_;
};

int main(int argc, char** argv)
{
    // ROS init
    ros::init(argc, argv, "nav_goal_qt_node");
    ros::NodeHandle nh;

    // ROS callback 用スレッド
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Qt アプリケーション
    int qt_argc = 0;
    QApplication app(qt_argc, nullptr);

    NavGoalWidget widget(nh);
    widget.show();

    int ret = app.exec();

    ros::shutdown();
    return ret;
}
