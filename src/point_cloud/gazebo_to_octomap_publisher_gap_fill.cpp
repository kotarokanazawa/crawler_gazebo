#include <ros/ros.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetModelProperties.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <fstream>
#include <sstream>
#include <cstdlib>
#include <algorithm>

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <osgDB/ReadFile>
#include <osg/Node>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/NodeVisitor>

#include <Eigen/Core>

// 追加: YAML(rosparam)の struct 読み取り
#include <xmlrpcpp/XmlRpcValue.h>

ros::Publisher octomap_pub;
ros::Publisher pointcloud_pub;
ros::Publisher mesh_pub;
ros::Publisher pointcloud_pre_gap_pub;
ros::Publisher pointcloud_voxel_pub;
ros::Publisher pointcloud_post_gap_pub;
ros::Publisher pointcloud_outlier_pub;
ros::Publisher pointcloud_overhang_pub;
ros::Publisher pointcloud_final_pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr unified_cloud(new pcl::PointCloud<pcl::PointXYZ>());
int g_gap_filter_max_neighbor_count = 8;
bool g_gap_filter_remove_interior = false;
bool g_outlier_filter_enabled = true;
int g_outlier_filter_min_neighbor_count = 1;
bool g_overhang_filter_enabled = true;
bool g_publish_debug_topics = false;
double g_final_upsample_resolution = 0.025;
int g_voxel_min_point_count = 1;

#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <cmath>

static inline int64_t q(double v, double step) {
  return (int64_t)std::llround(v / step);
}

static inline void hash_combine_u64(uint64_t& h, uint64_t v) {
  // boost::hash_combine と同等の定番
  h ^= v + 0x9e3779b97f4a7c15ULL + (h<<6) + (h>>2);
}

static inline uint64_t hash_str(const std::string& s) {
  // FNV-1a 64
  uint64_t h = 1469598103934665603ULL;
  for (unsigned char c : s) {
    h ^= (uint64_t)c;
    h *= 1099511628211ULL;
  }
  return h;
}

static inline uint64_t make_scene_signature(
    const std::vector<std::string>& model_names_sorted,
    const std::unordered_map<std::string, geometry_msgs::Pose>& pose_map,
    const std::unordered_map<std::string, Eigen::Vector3d>& scale_map,
    double pose_quant, double scale_quant)
{
  uint64_t h = 0xcbf29ce484222325ULL;

  for (const auto& name : model_names_sorted) {
    hash_combine_u64(h, hash_str(name));

    auto pit = pose_map.find(name);
    if (pit != pose_map.end()) {
      const auto& p = pit->second;
      hash_combine_u64(h, (uint64_t)q(p.position.x, pose_quant));
      hash_combine_u64(h, (uint64_t)q(p.position.y, pose_quant));
      hash_combine_u64(h, (uint64_t)q(p.position.z, pose_quant));
      hash_combine_u64(h, (uint64_t)q(p.orientation.x, pose_quant));
      hash_combine_u64(h, (uint64_t)q(p.orientation.y, pose_quant));
      hash_combine_u64(h, (uint64_t)q(p.orientation.z, pose_quant));
      hash_combine_u64(h, (uint64_t)q(p.orientation.w, pose_quant));
    } else {
      hash_combine_u64(h, 0xdeadbeefULL);
    }

    auto sit = scale_map.find(name);
    if (sit != scale_map.end()) {
      const auto& s = sit->second;
      hash_combine_u64(h, (uint64_t)q(s.x(), scale_quant));
      hash_combine_u64(h, (uint64_t)q(s.y(), scale_quant));
      hash_combine_u64(h, (uint64_t)q(s.z(), scale_quant));
    } else {
      hash_combine_u64(h, 0xcafebabeULL);
    }
  }
  return h;
}

struct GridCoord {
  int x;
  int y;
  int z;

  bool operator==(const GridCoord& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridCoordXY {
  int x;
  int y;

  bool operator==(const GridCoordXY& other) const {
    return x == other.x && y == other.y;
  }
};

struct GridCoordHash {
  std::size_t operator()(const GridCoord& coord) const {
    uint64_t h = 0xcbf29ce484222325ULL;
    hash_combine_u64(h, static_cast<uint64_t>(static_cast<int64_t>(coord.x)));
    hash_combine_u64(h, static_cast<uint64_t>(static_cast<int64_t>(coord.y)));
    hash_combine_u64(h, static_cast<uint64_t>(static_cast<int64_t>(coord.z)));
    return static_cast<std::size_t>(h);
  }
};

struct GridCoordXYHash {
  std::size_t operator()(const GridCoordXY& coord) const {
    uint64_t h = 0xcbf29ce484222325ULL;
    hash_combine_u64(h, static_cast<uint64_t>(static_cast<int64_t>(coord.x)));
    hash_combine_u64(h, static_cast<uint64_t>(static_cast<int64_t>(coord.y)));
    return static_cast<std::size_t>(h);
  }
};

static GridCoord pointToGrid(const pcl::PointXYZ& point, double resolution) {
  return GridCoord{
      static_cast<int>(std::floor(point.x / resolution)),
      static_cast<int>(std::floor(point.y / resolution)),
      static_cast<int>(std::floor(point.z / resolution))};
}

static pcl::PointXYZ gridToVoxelCenter(const GridCoord& coord, double resolution) {
  pcl::PointXYZ point;
  point.x = static_cast<float>((static_cast<double>(coord.x) + 0.5) * resolution);
  point.y = static_cast<float>((static_cast<double>(coord.y) + 0.5) * resolution);
  point.z = static_cast<float>((static_cast<double>(coord.z) + 0.5) * resolution);
  return point;
}

static void finalizeCloudMetadata(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  cloud->width = static_cast<std::uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = true;
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr gridCellsToCloud(
    const std::vector<GridCoord>& cells,
    double resolution) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->reserve(cells.size());
  for (const auto& cell : cells) {
    cloud->points.push_back(gridToVoxelCenter(cell, resolution));
  }
  finalizeCloudMetadata(cloud);
  return cloud;
}

static pcl::PointCloud<pcl::PointXYZ>::Ptr gridSetToCloud(
    const std::unordered_set<GridCoord, GridCoordHash>& cells,
    double resolution) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->reserve(cells.size());
  for (const auto& cell : cells) {
    cloud->points.push_back(gridToVoxelCenter(cell, resolution));
  }
  finalizeCloudMetadata(cloud);
  return cloud;
}

static void publishDebugPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    ros::Publisher& publisher) {
  if (!g_publish_debug_topics) {
    return;
  }
  finalizeCloudMetadata(cloud);
  sensor_msgs::PointCloud2 output_cloud;
  pcl::toROSMsg(*cloud, output_cloud);
  output_cloud.header.frame_id = "map";
  output_cloud.header.stamp = ros::Time::now();
  publisher.publish(output_cloud);
}

static std::vector<GridCoord> upsampleGridCells(
    const std::vector<GridCoord>& coarse_cells,
    double coarse_resolution,
    double fine_resolution) {
  if (fine_resolution >= coarse_resolution) {
    return coarse_cells;
  }

  if (coarse_cells.empty()) {
    return {};
  }

  std::unordered_set<GridCoord, GridCoordHash> coarse_set;
  coarse_set.reserve(coarse_cells.size());

  int min_x = coarse_cells.front().x;
  int min_y = coarse_cells.front().y;
  int min_z = coarse_cells.front().z;
  int max_x = coarse_cells.front().x;
  int max_y = coarse_cells.front().y;
  int max_z = coarse_cells.front().z;

  for (const auto& cell : coarse_cells) {
    coarse_set.insert(cell);
    min_x = std::min(min_x, cell.x);
    min_y = std::min(min_y, cell.y);
    min_z = std::min(min_z, cell.z);
    max_x = std::max(max_x, cell.x);
    max_y = std::max(max_y, cell.y);
    max_z = std::max(max_z, cell.z);
  }

  const pcl::PointXYZ min_center =
      gridToVoxelCenter(GridCoord{min_x, min_y, min_z}, coarse_resolution);
  const pcl::PointXYZ max_center =
      gridToVoxelCenter(GridCoord{max_x, max_y, max_z}, coarse_resolution);
  const double coarse_half = 0.5 * coarse_resolution;

  pcl::PointXYZ min_point;
  min_point.x = static_cast<float>(min_center.x - coarse_half);
  min_point.y = static_cast<float>(min_center.y - coarse_half);
  min_point.z = static_cast<float>(min_center.z - coarse_half);

  pcl::PointXYZ max_point;
  max_point.x = static_cast<float>(max_center.x + coarse_half - fine_resolution);
  max_point.y = static_cast<float>(max_center.y + coarse_half - fine_resolution);
  max_point.z = static_cast<float>(max_center.z + coarse_half - fine_resolution);

  const GridCoord min_fine = pointToGrid(min_point, fine_resolution);
  const GridCoord max_fine = pointToGrid(max_point, fine_resolution);

  std::vector<GridCoord> upsampled_cells;
  upsampled_cells.reserve(
      static_cast<std::size_t>(max_fine.x - min_fine.x + 1) *
      static_cast<std::size_t>(max_fine.y - min_fine.y + 1));

  auto sample_occupancy = [&](const GridCoord& fine_cell) -> double {
    const pcl::PointXYZ p = gridToVoxelCenter(fine_cell, fine_resolution);
    const double gx = p.x / coarse_resolution - 0.5;
    const double gy = p.y / coarse_resolution - 0.5;
    const double gz = p.z / coarse_resolution - 0.5;

    const int bx = static_cast<int>(std::floor(gx));
    const int by = static_cast<int>(std::floor(gy));
    const int bz = static_cast<int>(std::floor(gz));

    const double fx = gx - bx;
    const double fy = gy - by;
    const double fz = gz - bz;

    double occupancy = 0.0;
    for (int dx = 0; dx <= 1; ++dx) {
      const double wx = dx ? fx : (1.0 - fx);
      for (int dy = 0; dy <= 1; ++dy) {
        const double wy = dy ? fy : (1.0 - fy);
        for (int dz = 0; dz <= 1; ++dz) {
          const double wz = dz ? fz : (1.0 - fz);
          if (coarse_set.find(GridCoord{bx + dx, by + dy, bz + dz}) != coarse_set.end()) {
            occupancy += wx * wy * wz;
          }
        }
      }
    }
    return occupancy;
  };

  for (int x = min_fine.x; x <= max_fine.x; ++x) {
    for (int y = min_fine.y; y <= max_fine.y; ++y) {
      for (int z = min_fine.z; z <= max_fine.z; ++z) {
        const GridCoord fine_cell{x, y, z};
        if (sample_occupancy(fine_cell) >= 0.5) {
          upsampled_cells.push_back(fine_cell);
        }
      }
    }
  }

  return upsampled_cells;
}

static int countOccupiedNeighbors26(
    const std::unordered_set<GridCoord, GridCoordHash>& occupied,
    const GridCoord& center) {
  int count = 0;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        const GridCoord neighbor{center.x + dx, center.y + dy, center.z + dz};
        if (occupied.find(neighbor) != occupied.end()) {
          ++count;
        }
      }
    }
  }
  return count;
}

static int countOccupiedNeighbors8InPlane(
    const std::unordered_set<GridCoord, GridCoordHash>& occupied,
    const GridCoord& center) {
  int count = 0;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const GridCoord neighbor{center.x + dx, center.y + dy, center.z};
      if (occupied.find(neighbor) != occupied.end()) {
        ++count;
      }
    }
  }
  return count;
}

static std::vector<GridCoord> fillLinearGapsOnce(
    const std::unordered_set<GridCoord, GridCoordHash>& occupied) {
  std::unordered_set<GridCoord, GridCoordHash> fill_set;
  fill_set.reserve(occupied.size() / 4 + 1);

  for (const auto& cell : occupied) {
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }

        GridCoord candidate{cell.x + dx, cell.y + dy, cell.z};
        if (occupied.find(candidate) != occupied.end()) {
          continue;
        }

        if (countOccupiedNeighbors8InPlane(occupied, candidate) >= 6) {
          fill_set.insert(candidate);
        }
      }
    }
  }

  return std::vector<GridCoord>(fill_set.begin(), fill_set.end());
}

static std::vector<GridCoord> filterInteriorCells(
    const std::unordered_set<GridCoord, GridCoordHash>& occupied) {
  std::vector<GridCoord> filtered_cells;
  filtered_cells.reserve(occupied.size());

  for (const auto& cell : occupied) {
    if (countOccupiedNeighbors26(occupied, cell) > g_gap_filter_max_neighbor_count) {
      continue;
    }
    filtered_cells.push_back(cell);
  }

  return filtered_cells;
}

static std::vector<GridCoord> filterOutlierCells(
    const std::unordered_set<GridCoord, GridCoordHash>& occupied,
    int min_neighbor_count) {
  std::vector<GridCoord> filtered_cells;
  filtered_cells.reserve(occupied.size());

  for (const auto& cell : occupied) {
    if (countOccupiedNeighbors26(occupied, cell) < min_neighbor_count) {
      continue;
    }
    filtered_cells.push_back(cell);
  }

  return filtered_cells;
}

static std::vector<GridCoord> filterOverhangCells(
    const std::unordered_set<GridCoord, GridCoordHash>& occupied,
    double resolution,
    double vertical_gap_threshold) {
  std::unordered_map<GridCoordXY, std::vector<int>, GridCoordXYHash> z_columns;
  z_columns.reserve(occupied.size());

  for (const auto& cell : occupied) {
    z_columns[GridCoordXY{cell.x, cell.y}].push_back(cell.z);
  }

  std::vector<GridCoord> filtered_cells;
  filtered_cells.reserve(occupied.size());

  const int gap_threshold_steps =
      std::max(1, static_cast<int>(std::ceil(vertical_gap_threshold / resolution)));

  for (auto& entry : z_columns) {
    auto& z_values = entry.second;
    std::sort(z_values.begin(), z_values.end(), std::greater<int>());

    int keep_count = static_cast<int>(z_values.size());
    for (int i = 1; i < static_cast<int>(z_values.size()); ++i) {
      const int z_gap_steps = z_values[i - 1] - z_values[i];
      if (z_gap_steps >= gap_threshold_steps) {
        keep_count = i;
        break;
      }
    }

    for (int i = 0; i < keep_count; ++i) {
      filtered_cells.push_back(GridCoord{entry.first.x, entry.first.y, z_values[i]});
    }
  }

  return filtered_cells;
}


// ==========================
// 三角形内にランダム点を生成
// ==========================
void generateRandomPointsInTriangle(const pcl::Vertices& triangle,
                                    const pcl::PointCloud<pcl::PointXYZ>& cloud,
                                    int num_points,
                                    pcl::PointCloud<pcl::PointXYZ>& out_cloud) {
  const auto& p1 = cloud.points[triangle.vertices[0]];
  const auto& p2 = cloud.points[triangle.vertices[1]];
  const auto& p3 = cloud.points[triangle.vertices[2]];

  for (int i = 0; i < num_points; ++i) {
    double r1 = static_cast<double>(rand()) / RAND_MAX;
    double r2 = static_cast<double>(rand()) / RAND_MAX;
    double sqrt_r1 = sqrt(r1);

    pcl::PointXYZ point;
    point.x = (1 - sqrt_r1) * p1.x + (sqrt_r1 * (1 - r2)) * p2.x + (sqrt_r1 * r2) * p3.x;
    point.y = (1 - sqrt_r1) * p1.y + (sqrt_r1 * (1 - r2)) * p2.y + (sqrt_r1 * r2) * p3.y;
    point.z = (1 - sqrt_r1) * p1.z + (sqrt_r1 * (1 - r2)) * p2.z + (sqrt_r1 * r2) * p3.z;

    out_cloud.points.push_back(point);
  }
}


// ==========================
// config(rosparam) からインスタンス名(model_name)の scale を取得
// robocup_arena/objects/<model_name>/{scale_x,scale_y,scale_z}
// ==========================
static bool getScaleFromConfig(ros::NodeHandle& nh,
                               const std::string& instance_name,
                               Eigen::Vector3d& out_scale) {
  out_scale = Eigen::Vector3d(1.0, 1.0, 1.0);

  XmlRpc::XmlRpcValue objects;
  if (!nh.getParam("robocup_arena/objects", objects)) {
    return false;
  }
  if (objects.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_WARN("robocup_arena/objects is not a struct.");
    return false;
  }
  if (!objects.hasMember(instance_name)) {
    return false;  // そのインスタンスに設定がない(= 1,1,1)
  }

  XmlRpc::XmlRpcValue cfg = objects[instance_name];
  if (cfg.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    return false;
  }

  auto get_double = [&](const char* key, double def) -> double {
    if (!cfg.hasMember(key)) return def;
    XmlRpc::XmlRpcValue v = cfg[key];
    if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(v);
    if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)    return static_cast<int>(v);
    return def;
  };

  out_scale.x() = get_double("scale_x", 1.0);
  out_scale.y() = get_double("scale_y", 1.0);
  out_scale.z() = get_double("scale_z", 1.0);
  return true;
}


// ==========================
// Gazebo モデルから mesh パス & (SDF内)scale を取得
// ※注意: これはファイルに書かれた scale であり、spawn 時に XML を書換した場合は反映されません
// ==========================
std::pair<std::string, Eigen::Vector3d> getMeshPathAndScaleFromSDF(const std::string& model_name) {
  const char* gazebo_model_path_env = std::getenv("GAZEBO_MODEL_PATH");
  if (!gazebo_model_path_env) {
    ROS_WARN("GAZEBO_MODEL_PATH is not set.");
    return {"", Eigen::Vector3d(1.0, 1.0, 1.0)};
  }

  std::string gazebo_model_path(gazebo_model_path_env);
  std::vector<std::string> paths;
  std::istringstream ss(gazebo_model_path);
  std::string token;
  Eigen::Vector3d scale_vector(1.0, 1.0, 1.0);

  while (std::getline(ss, token, ':')) {
    paths.push_back(token);
  }

  for (const std::string& path : paths) {
    boost::filesystem::path model_dir = boost::filesystem::path(path) / model_name;

    if (boost::filesystem::exists(model_dir)) {
      for (const auto& entry : boost::filesystem::directory_iterator(model_dir)) {
        const std::string file_path = entry.path().string();
        if (boost::algorithm::ends_with(file_path, ".sdf") || boost::algorithm::ends_with(file_path, ".urdf")) {
          std::ifstream file(file_path);
          std::string line;
          std::string uri;

          while (std::getline(file, line)) {
            // <uri>
            if (line.find("<uri>") != std::string::npos) {
              std::string::size_type start = line.find("<uri>") + 5;
              std::string::size_type end = line.find("</uri>");
              uri = line.substr(start, end - start);

              if (boost::algorithm::starts_with(uri, "model://")) {
                std::string relative_path = uri.substr(8);
                boost::filesystem::path full_path = boost::filesystem::path(path) / relative_path;
                if (boost::filesystem::exists(full_path)) {
                  ROS_INFO("Mesh file found: %s", full_path.string().c_str());
                  uri = full_path.string();
                } else {
                  ROS_WARN("Mesh file not found: %s", full_path.string().c_str());
                  uri = "";
                }
              }
            }

            // <scale>
            if (line.find("<scale>") != std::string::npos) {
              std::string::size_type start = line.find("<scale>") + 7;
              std::string::size_type end = line.find("</scale>");
              std::string scale_str = line.substr(start, end - start);

              std::istringstream ss2(scale_str);
              double sx, sy, sz;
              if (ss2 >> sx >> sy >> sz) {
                scale_vector = Eigen::Vector3d(sx, sy, sz);
              } else {
                try {
                  double s = std::stod(scale_str);
                  scale_vector = Eigen::Vector3d(s, s, s);
                } catch (...) {
                  scale_vector = Eigen::Vector3d(1.0, 1.0, 1.0);
                }
              }
            }
          }

          if (!uri.empty()) {
            return {uri, scale_vector};
          }
        }
      }
    }
  }

  ROS_WARN("Model not found in GAZEBO_MODEL_PATH: %s", model_name.c_str());
  return {"", Eigen::Vector3d(1.0, 1.0, 1.0)};
}


// ==========================
// 可視化用 Marker
// ==========================
visualization_msgs::Marker publishMeshFromDAE(const std::string& dae_file,
                                             const geometry_msgs::Pose& pose,
                                             Eigen::Vector3d scale_vector,
                                             int id) {
  visualization_msgs::Marker mesh_marker;
  mesh_marker.header.frame_id = "map";
  mesh_marker.header.stamp = ros::Time::now();
  mesh_marker.ns = "mesh_namespace";
  mesh_marker.id = id;
  mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  mesh_marker.action = visualization_msgs::Marker::ADD;
  mesh_marker.mesh_resource = "file://" + dae_file;
  mesh_marker.pose = pose;
  mesh_marker.scale.x = scale_vector.x();
  mesh_marker.scale.y = scale_vector.y();
  mesh_marker.scale.z = scale_vector.z();
  mesh_marker.color.a = 1.0;
  mesh_marker.color.r = 0.6;
  mesh_marker.color.g = 0.3;
  mesh_marker.color.b = 0.0;
  return mesh_marker;
}

void publishMeshesFromDAE(const std::vector<std::string>& dae_files,
                          const std::vector<geometry_msgs::Pose>& poses,
                          const std::vector<Eigen::Vector3d>& scale_factors) {
  if (dae_files.size() != poses.size() || dae_files.size() != scale_factors.size()) {
    ROS_ERROR("The sizes of dae_files, poses, and scale_factors must be the same.");
    return;
  }
  visualization_msgs::MarkerArray marker_array;
  for (size_t i = 0; i < dae_files.size(); ++i) {
    ROS_INFO("Publishing mesh: %s", dae_files[i].c_str());
    marker_array.markers.push_back(
        publishMeshFromDAE(dae_files[i], poses[i], scale_factors[i], static_cast<int>(i)));
  }
  mesh_pub.publish(marker_array);
}


// ==========================
// DAE → Mesh
// ==========================
pcl::PolygonMesh ConvertDaeToMesh(const std::string& dae_file) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PolygonMesh polygon_mesh;

  osg::ref_ptr<osg::Node> root = osgDB::readNodeFile(dae_file);
  if (!root) {
    ROS_ERROR("Failed to load DAE file: %s", dae_file.c_str());
    return polygon_mesh;
  }

  struct GeometryCollector : public osg::NodeVisitor {
    pcl::PointCloud<pcl::PointXYZ>& cloud_ref;
    std::vector<pcl::Vertices>& polygons_ref;

    GeometryCollector(pcl::PointCloud<pcl::PointXYZ>& cloud,
                      std::vector<pcl::Vertices>& polygons)
      : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
        cloud_ref(cloud), polygons_ref(polygons) {}

    void apply(osg::Geode& geode) override {
      for (unsigned int i = 0; i < geode.getNumDrawables(); ++i) {
        osg::Geometry* geometry = dynamic_cast<osg::Geometry*>(geode.getDrawable(i));
        if (geometry) processGeometry(geometry);
      }
    }

    void processGeometry(osg::Geometry* geometry) {
      const osg::Vec3Array* vec3Array =
          dynamic_cast<const osg::Vec3Array*>(geometry->getVertexArray());

      if (vec3Array) {
        for (const osg::Vec3& vertex : *vec3Array) {
          pcl::PointXYZ point;
          point.x = vertex.x();
          point.y = vertex.y();
          point.z = vertex.z();
          cloud_ref.push_back(point);
        }
      }

      for (unsigned int i = 0; i < geometry->getNumPrimitiveSets(); ++i) {
        const osg::DrawElementsUInt* drawElements =
            dynamic_cast<const osg::DrawElementsUInt*>(geometry->getPrimitiveSet(i));

        if (drawElements) {
          for (unsigned int j = 0; j + 2 < drawElements->getNumIndices(); j += 3) {
            pcl::Vertices vertices;
            vertices.vertices.push_back(drawElements->at(j));
            vertices.vertices.push_back(drawElements->at(j + 1));
            vertices.vertices.push_back(drawElements->at(j + 2));
            polygons_ref.push_back(vertices);
          }
        }
      }
    }
  };

  GeometryCollector collector(cloud, polygon_mesh.polygons);
  root->accept(collector);

  pcl::toPCLPointCloud2(cloud, polygon_mesh.cloud);
  ROS_INFO("Converted DAE file to polygon mesh with %zu points and %zu polygons.",
           cloud.size(), polygon_mesh.polygons.size());
  return polygon_mesh;
}


// ==========================
// PointCloud → Octomap
// ==========================
void publishPointCloud(double resolution) {
  constexpr double kOverhangGapThreshold = 0.08;
  sensor_msgs::PointCloud2 output_cloud;

  publishDebugPointCloud(unified_cloud, pointcloud_pre_gap_pub);

  std::unordered_map<GridCoord, int, GridCoordHash> voxel_counts;
  voxel_counts.reserve(unified_cloud->points.size());
  for (const auto& point : unified_cloud->points) {
    if (!pcl::isFinite(point)) {
      continue;
    }
    ++voxel_counts[pointToGrid(point, resolution)];
  }

  std::unordered_set<GridCoord, GridCoordHash> occupied_cells;
  occupied_cells.reserve(voxel_counts.size());
  for (const auto& entry : voxel_counts) {
    if (entry.second < g_voxel_min_point_count) {
      continue;
    }
    occupied_cells.insert(entry.first);
  }
  unified_cloud = gridSetToCloud(occupied_cells, resolution);
  publishDebugPointCloud(unified_cloud, pointcloud_voxel_pub);

  const std::vector<GridCoord> post_voxel_filled_cells = fillLinearGapsOnce(occupied_cells);
  const std::size_t post_gap_filter_added = post_voxel_filled_cells.size();
  for (const auto& cell : post_voxel_filled_cells) {
    occupied_cells.insert(cell);
  }
  publishDebugPointCloud(gridSetToCloud(occupied_cells, resolution), pointcloud_post_gap_pub);

  std::vector<GridCoord> outlier_filtered_cells;
  if (g_outlier_filter_enabled) {
    outlier_filtered_cells =
        filterOutlierCells(occupied_cells, g_outlier_filter_min_neighbor_count);
  } else {
    outlier_filtered_cells.assign(occupied_cells.begin(), occupied_cells.end());
  }

  std::unordered_set<GridCoord, GridCoordHash> outlier_filtered_set;
  outlier_filtered_set.reserve(outlier_filtered_cells.size());
  for (const auto& cell : outlier_filtered_cells) {
    outlier_filtered_set.insert(cell);
  }

  publishDebugPointCloud(gridCellsToCloud(outlier_filtered_cells, resolution),
                         pointcloud_outlier_pub);

  std::vector<GridCoord> overhang_filtered_cells;
  if (g_overhang_filter_enabled) {
    overhang_filtered_cells =
        filterOverhangCells(outlier_filtered_set, resolution, kOverhangGapThreshold);
  } else {
    overhang_filtered_cells = outlier_filtered_cells;
  }
  publishDebugPointCloud(gridCellsToCloud(overhang_filtered_cells, resolution),
                         pointcloud_overhang_pub);

  std::vector<GridCoord> filtered_cells;
  if (g_gap_filter_remove_interior) {
    std::unordered_set<GridCoord, GridCoordHash> overhang_filtered_set;
    overhang_filtered_set.reserve(overhang_filtered_cells.size());
    for (const auto& cell : overhang_filtered_cells) {
      overhang_filtered_set.insert(cell);
    }
    filtered_cells = filterInteriorCells(overhang_filtered_set);
  } else {
    filtered_cells = overhang_filtered_cells;
  }

  const std::vector<GridCoord> upsampled_cells =
      upsampleGridCells(filtered_cells, resolution, g_final_upsample_resolution);

  pcl::PointCloud<pcl::PointXYZ>::Ptr completed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  completed_cloud->reserve(upsampled_cells.size());

  for (const auto& cell : upsampled_cells) {
    completed_cloud->points.push_back(gridToVoxelCenter(cell, g_final_upsample_resolution));
  }
  unified_cloud = completed_cloud;
  publishDebugPointCloud(unified_cloud, pointcloud_final_pub);
  finalizeCloudMetadata(unified_cloud);

  pcl::toROSMsg(*unified_cloud, output_cloud);
  output_cloud.header.frame_id = "map";
  output_cloud.header.stamp = ros::Time::now();
  pointcloud_pub.publish(output_cloud);

  octomap::OcTree gtree(g_final_upsample_resolution);
  for (const auto& point : unified_cloud->points) {
    if (pcl::isFinite(point)) {
      gtree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
    }
  }
  gtree.updateInnerOccupancy();

  octomap_msgs::Octomap octomap_msg;
  octomap_msgs::fullMapToMsg(gtree, octomap_msg);
  octomap_msg.header.frame_id = "map";
  octomap_msg.header.stamp = ros::Time::now();
  octomap_pub.publish(octomap_msg);

  ROS_INFO("PointCloud and Octomap published successfully. voxel_resolution=%.3f final_upsample_resolution=%.3f gap_filter_added=%zu outlier_removed=%zu overhang_removed=%zu filtered_total=%zu upsampled_total=%zu",
           resolution,
           g_final_upsample_resolution,
           post_gap_filter_added,
           occupied_cells.size() - outlier_filtered_cells.size(),
           outlier_filtered_cells.size() - overhang_filtered_cells.size(),
           overhang_filtered_cells.size() - filtered_cells.size(),
           upsampled_cells.size());
}

void resetUinfiedCloud() {
  unified_cloud->clear();
}


// ==========================
// PointCloud 生成
// ==========================
void generatePointCloud(const std::vector<std::string>& model_files,
                        const std::vector<geometry_msgs::Pose>& poses,
                        const std::vector<Eigen::Vector3d>& scale_factors,
                        double resolution) {
  for (size_t i = 0; i < model_files.size(); ++i) {
    const std::string& file = model_files[i];
    std::string file_extension = file.substr(file.find_last_of(".") + 1);

    pcl::PolygonMesh mesh;
    if (file_extension == "stl") {
      if (pcl::io::loadPolygonFileSTL(file, mesh) == 0) {
        ROS_WARN("Failed to load STL file: %s. Skipping.", file.c_str());
        continue;
      }
      ROS_INFO("STL file %s loaded.", file.c_str());
    } else if (file_extension == "dae") {
      mesh = ConvertDaeToMesh(file);
      if (mesh.cloud.data.empty()) {
        ROS_WARN("DAE file %s produced empty cloud. Skipping.", file.c_str());
        continue;
      }
      ROS_INFO("DAE file %s loaded.", file.c_str());
    } else {
      ROS_WARN("Unsupported file: %s. Skipping.", file.c_str());
      continue;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud);

    Eigen::Vector3d scale_factor = (i < scale_factors.size()) ? scale_factors[i]
                                                              : Eigen::Vector3d(1.0, 1.0, 1.0);

    // 既存の補正（必要なら維持）
    if (file_extension == "dae") {
      scale_factor *= 0.1;  // DAE の補正
    }

    ROS_INFO("Scaling factor: %f %f %f", scale_factor.x(), scale_factor.y(), scale_factor.z());

    for (auto& point : cloud) {
      point.x *= scale_factor.x();
      point.y *= scale_factor.y();
      point.z *= scale_factor.z();
    }

    pcl::PointCloud<pcl::PointXYZ> random_points;
    for (const auto& triangle : mesh.polygons) {
      generateRandomPointsInTriangle(triangle, cloud, 1200, random_points);
    }

    if (i < poses.size()) {
      Eigen::Affine3f transform = Eigen::Affine3f::Identity();
      const auto& pose = poses[i];
      transform.translation() << pose.position.x, pose.position.y, pose.position.z;
      Eigen::Quaternionf rotation(pose.orientation.w, pose.orientation.x,
                                  pose.orientation.y, pose.orientation.z);
      transform.rotate(rotation);
      pcl::transformPointCloud(random_points, random_points, transform);
    }

    *unified_cloud += random_points;
  }

  // 地面の平面追加
  double plane_size = 10.0;
  double plane_resolution = 0.04;
  for (double x = -plane_size / 2; x <= plane_size / 2; x += plane_resolution) {
    for (double y = -plane_size / 2; y <= plane_size / 2; y += plane_resolution) {
      pcl::PointXYZ point;
      point.x = x;
      point.y = y;
      point.z = 0.0;
      unified_cloud->points.push_back(point);
    }
  }

  publishPointCloud(resolution);
}


// ==========================
// Gazebo からモデル取得
// ==========================
static bool publishMeshesFromGazeboModelsIfChanged(
    ros::NodeHandle& nh,
    double resolution,
    uint64_t& inout_last_signature)
{
  ros::ServiceClient world_properties_client =
      nh.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
  gazebo_msgs::GetWorldProperties world_properties;

  if (!world_properties_client.call(world_properties)) {
    ROS_ERROR("Failed to call /gazebo/get_world_properties service");
    return false;
  }

  // まず「対象モデル」の pose/scale を取得して署名を作る
  std::vector<std::string> target_names;
  target_names.reserve(world_properties.response.model_names.size());

  std::unordered_map<std::string, geometry_msgs::Pose> pose_map;
  std::unordered_map<std::string, Eigen::Vector3d> scale_map;

  ros::ServiceClient model_state_client =
      nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

  for (const std::string& model_name : world_properties.response.model_names) {
    if (model_name == "ground_plane" || model_name == "crawler") continue;

    gazebo_msgs::GetModelState get_model_state;
    get_model_state.request.model_name = model_name;
    if (!model_state_client.call(get_model_state)) {
      ROS_WARN("Failed to get state for model: %s", model_name.c_str());
      continue;
    }
    pose_map[model_name] = get_model_state.response.pose;

    // model:// のフォルダ名に合わせるため、末尾 _<digits> / _clone を落とす
    std::string modified_model_name = model_name;
    std::size_t underscore_pos = model_name.find_last_of('_');
    if (underscore_pos != std::string::npos) {
      std::string suffix = model_name.substr(underscore_pos + 1);
      if (suffix == "clone" || std::all_of(suffix.begin(), suffix.end(), ::isdigit)) {
        modified_model_name = model_name.substr(0, underscore_pos);
      }
    }

    auto mesh_file_path = getMeshPathAndScaleFromSDF(modified_model_name);
    if (mesh_file_path.first.empty()) {
      // mesh取れないものは対象外（署名にも入れない）
      continue;
    }

    Eigen::Vector3d cfg_scale(1.0, 1.0, 1.0);
    getScaleFromConfig(nh, model_name, cfg_scale);

    Eigen::Vector3d final_scale = mesh_file_path.second.cwiseProduct(cfg_scale);

    // あなたの既存補正（DAE 0.1 倍）を署名にも反映しておく（重要）
    const std::string& file = mesh_file_path.first;
    std::string ext = file.substr(file.find_last_of(".") + 1);
    if (ext == "dae") {
      final_scale *= 0.1;
    }

    scale_map[model_name] = final_scale;

    target_names.push_back(model_name);
  }

  std::sort(target_names.begin(), target_names.end());

  double pose_quant, scale_quant;
  nh.param("scene_change/pose_quant",  pose_quant,  1e-4);  // 0.1mm 相当（好みで）
  nh.param("scene_change/scale_quant", scale_quant, 1e-6);

  const uint64_t sig = make_scene_signature(target_names, pose_map, scale_map,
                                            pose_quant, scale_quant);

  if (sig == inout_last_signature) {
    // 変化なし：何も publish しない
    return false;
  }

  // ここから先は「変化あり」のときだけ実行する
  inout_last_signature = sig;

  std::vector<std::string> dae_files;
  std::vector<geometry_msgs::Pose> poses;
  std::vector<Eigen::Vector3d> scale_factors;

  dae_files.reserve(target_names.size());
  poses.reserve(target_names.size());
  scale_factors.reserve(target_names.size());

  // target_names を使って順序固定で生成（デバッグしやすい）
  for (const std::string& model_name : target_names) {
    // もう一度メッシュパスが必要なので再取得する（軽くしたいなら前段で保持してもよい）
    std::string modified_model_name = model_name;
    std::size_t underscore_pos = model_name.find_last_of('_');
    if (underscore_pos != std::string::npos) {
      std::string suffix = model_name.substr(underscore_pos + 1);
      if (suffix == "clone" || std::all_of(suffix.begin(), suffix.end(), ::isdigit)) {
        modified_model_name = model_name.substr(0, underscore_pos);
      }
    }

    auto mesh_file_path = getMeshPathAndScaleFromSDF(modified_model_name);
    if (mesh_file_path.first.empty()) continue;

    dae_files.push_back(mesh_file_path.first);
    poses.push_back(pose_map[model_name]);
    scale_factors.push_back(scale_map[model_name]);
  }

  if (!dae_files.empty()) {
    publishMeshesFromDAE(dae_files, poses, scale_factors);
  }

  resetUinfiedCloud();  // 追加：前回の点群が残ると差分更新時に混ざるので，ここでクリア推奨
  generatePointCloud(dae_files, poses, scale_factors, resolution);

  ROS_INFO("Scene changed. Published mesh/pointcloud/octomap. signature=0x%016llx",
           (unsigned long long)inout_last_signature);

  return true;
}

// ==========================
// main
// ==========================
int main(int argc, char** argv) {
  ros::init(argc, argv, "gazebo_to_octomap_publisher_gap_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  octomap_pub = nh.advertise<octomap_msgs::Octomap>("octomap", 1, true);
  pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("octomap_pointcloud", 1, true);
  pointcloud_pre_gap_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/octomap_pointcloud/raw_input", 1, true);
  pointcloud_voxel_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/octomap_pointcloud/voxel", 1, true);
  pointcloud_post_gap_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/octomap_pointcloud/gap_filtered", 1, true);
  pointcloud_outlier_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/octomap_pointcloud/outlier_filtered", 1, true);
  pointcloud_overhang_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/octomap_pointcloud/overhang_filtered", 1, true);
  pointcloud_final_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/octomap_pointcloud/final_upsampled", 1, true);
  mesh_pub = nh.advertise<visualization_msgs::MarkerArray>("mesh_marker", 1, true);

  double resolution;
  pnh.param<double>("resolution", resolution, 0.03);
  pnh.param<bool>("debug_publish/enabled", g_publish_debug_topics, false);
  pnh.param<double>("upsample/final_resolution", g_final_upsample_resolution, 0.025);
  pnh.param<int>("voxel_filter/min_point_count", g_voxel_min_point_count, 1);
  pnh.param<bool>("outlier_filter/enabled", g_outlier_filter_enabled, true);
  pnh.param<int>("outlier_filter/min_neighbor_count", g_outlier_filter_min_neighbor_count, 1);
  pnh.param<bool>("overhang_filter/enabled", g_overhang_filter_enabled, true);
  pnh.param<bool>("gap_filter/remove_interior", g_gap_filter_remove_interior, false);
  pnh.param<int>("gap_filter/max_neighbor_count", g_gap_filter_max_neighbor_count, 8);

  uint64_t last_sig = 0;         // 追加
  bool first = true;
  int loop_time = 0;
  int loop_time_threshold = 5; // 変更: ループタイムの閾値を定数化
  bool changed = false;

  ros::Rate loop_rate(0.5);
  while (ros::ok()) {
    if (first) { last_sig = 0; first = false; }
    if(publishMeshesFromGazeboModelsIfChanged(nh, resolution, last_sig)){     
        if(loop_time==0){
          changed =true;
        }
        loop_time++;
    }else{
    }

    if(changed && loop_time<loop_time_threshold){
      last_sig=0;
    }
    if(changed && loop_time>=loop_time_threshold){
      changed=false;
      loop_time=0;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
