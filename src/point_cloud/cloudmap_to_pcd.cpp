#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class CloudmapToPcd : public rclcpp::Node
{
public:
  CloudmapToPcd()
  : Node("cloudmap_to_pcd")
  {
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/loaded_pointcloud");
    output_path_ = this->declare_parameter<std::string>("output_path", defaultOutputPath());
    save_once_ = this->declare_parameter<bool>("save_once", true);
    binary_ = this->declare_parameter<bool>("binary", true);
    compressed_ = this->declare_parameter<bool>("compressed", false);
    overwrite_ = this->declare_parameter<bool>("overwrite", true);
    queue_depth_ = this->declare_parameter<int>("queue_depth", 1);
    crop_xy_ = this->declare_parameter<bool>("crop_xy", false);
    min_x_ = this->declare_parameter<double>("min_x", -std::numeric_limits<double>::max());
    max_x_ = this->declare_parameter<double>("max_x", std::numeric_limits<double>::max());
    min_y_ = this->declare_parameter<double>("min_y", -std::numeric_limits<double>::max());
    max_y_ = this->declare_parameter<double>("max_y", std::numeric_limits<double>::max());
    const auto durability = this->declare_parameter<std::string>("durability", "volatile");

    if (crop_xy_ && (min_x_ > max_x_ || min_y_ > max_y_)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid crop range: x=[%.3f, %.3f], y=[%.3f, %.3f]. Crop is disabled.",
        min_x_,
        max_x_,
        min_y_,
        max_y_);
      crop_xy_ = false;
    }

    rclcpp::QoS qos(static_cast<size_t>(std::max(queue_depth_, 1)));
    qos.reliable();
    if (durability == "transient_local") {
      qos.transient_local();
    } else if (durability == "system_default") {
      qos.durability(rclcpp::DurabilityPolicy::SystemDefault);
    } else {
      qos.durability_volatile();
    }

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      qos,
      std::bind(&CloudmapToPcd::pointCloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Waiting for PointCloud2 on [%s], saving to [%s]",
      input_topic_.c_str(),
      output_path_.c_str());
    if (crop_xy_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Cropping XY range: x=[%.3f, %.3f], y=[%.3f, %.3f]",
        min_x_,
        max_x_,
        min_y_,
        max_y_);
    }
  }

private:
  static std::string defaultOutputPath()
  {
    return "package://crawler_gazebo/pcd/cloudmap.pcd";
  }

  static std::filesystem::path resolveOutputPath(const std::string& value)
  {
    constexpr char prefix[] = "package://";
    if (value.rfind(prefix, 0) != 0) {
      return std::filesystem::path(value);
    }

    const std::string package_path = value.substr(sizeof(prefix) - 1);
    const auto slash = package_path.find('/');
    const std::string package_name = slash == std::string::npos ?
      package_path : package_path.substr(0, slash);
    const std::string relative_path = slash == std::string::npos ?
      "" : package_path.substr(slash + 1);
    return packageRoot(package_name) / relative_path;
  }

  static std::filesystem::path packageRoot(const std::string& package_name)
  {
    const auto share = std::filesystem::path(
      ament_index_cpp::get_package_share_directory(package_name));
    if (package_name != "crawler_gazebo") {
      return share;
    }

    const char* env_source = std::getenv("CRAWLER_GAZEBO_SOURCE_DIR");
    if (env_source != nullptr && env_source[0] != '\0') {
      const auto source = std::filesystem::path(env_source);
      if (std::filesystem::exists(source / "package.xml")) {
        return source;
      }
    }

    for (auto current = share; !current.empty(); current = current.parent_path()) {
      if (current.filename() != "install") {
        if (current == current.root_path()) {
          break;
        }
        continue;
      }

      const auto source_root = current.parent_path() / "src";
      if (!std::filesystem::exists(source_root)) {
        return share;
      }
      for (const auto& entry : std::filesystem::recursive_directory_iterator(source_root)) {
        if (!entry.is_regular_file() || entry.path().filename() != "package.xml") {
          continue;
        }
        if (entry.path().parent_path().filename() == "crawler_gazebo") {
          return entry.path().parent_path();
        }
      }
      return share;
    }

    return share;
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (save_once_ && saved_count_ > 0) {
      return;
    }

    const auto path = nextOutputPath();
    if (!overwrite_ && std::filesystem::exists(path)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Output file already exists and overwrite=false: %s",
        path.string().c_str());
      return;
    }

    if (path.has_parent_path()) {
      std::error_code error;
      std::filesystem::create_directories(path.parent_path(), error);
      if (error) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to create output directory [%s]: %s",
          path.parent_path().string().c_str(),
          error.message().c_str());
        return;
      }
    }

    pcl::PCLPointCloud2 cloud;
    pcl_conversions::toPCL(*msg, cloud);
    const auto input_points = static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);

    if (crop_xy_ && !cropCloudXY(cloud)) {
      return;
    }

    pcl::PCDWriter writer;
    int result = -1;
    if (compressed_) {
      result = writer.writeBinaryCompressed(path.string(), cloud);
    } else if (binary_) {
      result = writer.writeBinary(path.string(), cloud);
    } else {
      result = writer.writeASCII(path.string(), cloud);
    }

    if (result != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to save PCD: %s", path.string().c_str());
      return;
    }

    ++saved_count_;
    RCLCPP_INFO(
      this->get_logger(),
      "Saved %zu/%zu points to %s",
      static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height),
      input_points,
      path.string().c_str());

    if (save_once_) {
      rclcpp::shutdown();
    }
  }

  std::filesystem::path nextOutputPath() const
  {
    const std::filesystem::path base = resolveOutputPath(output_path_);
    if (save_once_) {
      return base;
    }

    const auto parent = base.parent_path();
    const auto stem = base.stem().string();
    const auto extension = base.extension().empty() ? ".pcd" : base.extension().string();
    const auto filename = stem + "_" + std::to_string(saved_count_) + extension;
    return parent.empty() ? std::filesystem::path(filename) : parent / filename;
  }

  int fieldIndex(const pcl::PCLPointCloud2& cloud, const std::string& name) const
  {
    for (size_t i = 0; i < cloud.fields.size(); ++i) {
      if (cloud.fields[i].name == name) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  bool readFieldAsDouble(
    const std::uint8_t* point_data,
    const pcl::PCLPointField& field,
    double& value) const
  {
    const auto* data = point_data + field.offset;
    switch (field.datatype) {
      case pcl::PCLPointField::FLOAT32: {
          float v = 0.0F;
          std::memcpy(&v, data, sizeof(v));
          value = static_cast<double>(v);
          return true;
        }
      case pcl::PCLPointField::FLOAT64: {
          double v = 0.0;
          std::memcpy(&v, data, sizeof(v));
          value = v;
          return true;
        }
      case pcl::PCLPointField::INT8:
        value = static_cast<double>(*reinterpret_cast<const std::int8_t*>(data));
        return true;
      case pcl::PCLPointField::UINT8:
        value = static_cast<double>(*reinterpret_cast<const std::uint8_t*>(data));
        return true;
      case pcl::PCLPointField::INT16: {
          std::int16_t v = 0;
          std::memcpy(&v, data, sizeof(v));
          value = static_cast<double>(v);
          return true;
        }
      case pcl::PCLPointField::UINT16: {
          std::uint16_t v = 0;
          std::memcpy(&v, data, sizeof(v));
          value = static_cast<double>(v);
          return true;
        }
      case pcl::PCLPointField::INT32: {
          std::int32_t v = 0;
          std::memcpy(&v, data, sizeof(v));
          value = static_cast<double>(v);
          return true;
        }
      case pcl::PCLPointField::UINT32: {
          std::uint32_t v = 0;
          std::memcpy(&v, data, sizeof(v));
          value = static_cast<double>(v);
          return true;
        }
      default:
        return false;
    }
  }

  bool cropCloudXY(pcl::PCLPointCloud2& cloud) const
  {
    const int x_index = fieldIndex(cloud, "x");
    const int y_index = fieldIndex(cloud, "y");
    if (x_index < 0 || y_index < 0) {
      RCLCPP_ERROR(this->get_logger(), "Cannot crop cloud: x/y fields were not found.");
      return false;
    }

    const auto& x_field = cloud.fields[static_cast<size_t>(x_index)];
    const auto& y_field = cloud.fields[static_cast<size_t>(y_index)];
    pcl::PCLPointCloud2 cropped = cloud;
    cropped.height = 1;
    cropped.width = 0;
    cropped.row_step = 0;
    cropped.data.clear();
    cropped.data.reserve(cloud.data.size());

    for (uint32_t row = 0; row < cloud.height; ++row) {
      for (uint32_t col = 0; col < cloud.width; ++col) {
        const size_t offset =
          static_cast<size_t>(row) * static_cast<size_t>(cloud.row_step) +
          static_cast<size_t>(col) * static_cast<size_t>(cloud.point_step);
        if (offset + cloud.point_step > cloud.data.size()) {
          continue;
        }

        const auto* point_data = cloud.data.data() + offset;
        double x = 0.0;
        double y = 0.0;
        if (!readFieldAsDouble(point_data, x_field, x) || !readFieldAsDouble(point_data, y_field, y)) {
          RCLCPP_ERROR(this->get_logger(), "Cannot crop cloud: x/y fields use unsupported datatypes.");
          return false;
        }
        if (!std::isfinite(x) || !std::isfinite(y)) {
          continue;
        }
        if (x < min_x_ || x > max_x_ || y < min_y_ || y > max_y_) {
          continue;
        }

        cropped.data.insert(cropped.data.end(), point_data, point_data + cloud.point_step);
        ++cropped.width;
      }
    }

    cropped.row_step = cropped.point_step * cropped.width;
    cropped.is_dense = cloud.is_dense;
    cloud = std::move(cropped);
    return true;
  }

  std::string input_topic_;
  std::string output_path_;
  bool save_once_{true};
  bool binary_{true};
  bool compressed_{false};
  bool overwrite_{true};
  int queue_depth_{1};
  bool crop_xy_{false};
  double min_x_{0.0};
  double max_x_{0.0};
  double min_y_{0.0};
  double max_y_{0.0};
  size_t saved_count_{0};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudmapToPcd>());
  rclcpp::shutdown();
  return 0;
}
