#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Trigger.h>   // Triggerサービスを利用

class PointCloudToGridMap
{
public:
    PointCloudToGridMap(ros::NodeHandle& nh)
    {
        // パラメータを取得
        double resolution;
        std::string frame_id;
        double max_x, max_y;

        nh.param("resolution", resolution, 0.05);
        nh.param("frame_id", frame_id, std::string("map"));
        nh.param("max_x", max_x, 10.0);
        nh.param("max_y", max_y, 10.0);
        nh.param("csv_save_path", csv_save_path_, std::string("/tmp/gridmap.csv"));

        pointcloud_subscriber_ = nh.subscribe("/octomap_pointcloud", 1, &PointCloudToGridMap::pointCloudCallback, this);
        gridmap_publisher_ = nh.advertise<grid_map_msgs::GridMap>("/grid_map", 1, true);

        // GridMap 初期化
        grid_map_.setFrameId(frame_id);
        grid_map_.setGeometry(grid_map::Length(max_x, max_y), resolution);
        grid_map_.add("elevation", 0.0);

        // サービスサーバー作成
        save_service_ = nh.advertiseService("save_gridmap_csv", &PointCloudToGridMap::saveGridMapCallback, this);

        ROS_INFO("PointCloudToGridMap initialized. Call service /save_gridmap_csv to save CSV.");
    }

private:
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    for (const auto& point : cloud.points)
    {
        if (!std::isfinite(point.z)) continue;

        grid_map::Index index;
        if (grid_map_.getIndex(grid_map::Position(point.x, point.y), index))
        {
            float& cell = grid_map_.at("elevation", index);

            // 未初期化（NaN）の場合はそのまま代入
            if (!std::isfinite(cell))
            {
                cell = point.z;
            }
            // 既に値がある場合は「より高い値のみ」採用
            else if (point.z > cell)
            {
                cell = point.z;
            }
        }
    }

    grid_map_msgs::GridMap grid_map_msg;
    grid_map::GridMapRosConverter::toMessage(grid_map_, grid_map_msg);
    grid_map_msg.info.header.stamp = ros::Time::now();
    grid_map_msg.info.header.frame_id = grid_map_.getFrameId();

    gridmap_publisher_.publish(grid_map_msg);
}
    bool saveGridMapCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
    {
        std::ofstream file(csv_save_path_);
        if (!file.is_open()) {
            res.success = false;
            res.message = "Failed to open file: " + csv_save_path_;
            return true;
        }

        file << "x,y,elevation\n";
        for (grid_map::GridMapIterator it(grid_map_); !it.isPastEnd(); ++it)
        {
            grid_map::Index index(*it);
            if (grid_map_.isValid(index, "elevation")) {
                grid_map::Position pos;
                grid_map_.getPosition(index, pos);
                double value = grid_map_.at("elevation", index);
                file << pos.x() << "," << pos.y() << "," << value << "\n";
            }
        }

        file.close();
        res.success = true;
        res.message = "Saved gridmap CSV to " + csv_save_path_;
        ROS_INFO("%s", res.message.c_str());
        return true;
    }

    ros::Subscriber pointcloud_subscriber_;
    ros::Publisher gridmap_publisher_;
    ros::ServiceServer save_service_;
    grid_map::GridMap grid_map_;
    std::string csv_save_path_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_to_gridmap");
    ros::NodeHandle nh("~");

    PointCloudToGridMap converter(nh);

    ros::spin();

    return 0;
}
