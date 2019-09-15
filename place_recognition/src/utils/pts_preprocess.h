#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include "PosesPts.h"
#include "pts_filter.h"

#define PTS_HIST 6
#define INIT_FRAME 30

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

void read_poses_pts(const std::string &poses_history_file_name,
                    const std::string &pts_history_file_name,
                    std::vector<IDPose *> &poses_history,
                    std::vector<IDPtIntensity *> &pts_history) {
  // read poses
  std::ifstream poses_history_file(poses_history_file_name);
  while (true) {
    int iid;
    if (!(poses_history_file >> iid))
      break;
    Eigen::Matrix<double, 3, 4> w2c;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 4; j++) {
        if (!(poses_history_file >> w2c(i, j)))
          break;
      }
    }
    poses_history.push_back(new IDPose(iid, w2c));
  }
  poses_history_file.close();

  // read points
  std::ifstream pts_history_file(pts_history_file_name);
  while (true) {
    int iid;
    Eigen::Vector3d p;
    float it;
    if (!(pts_history_file >> iid >> p(0) >> p(1) >> p(2) >> it))
      break;
    pts_history.push_back(new IDPtIntensity(iid, p, it));
  }
  pts_history_file.close();
}

void get_spherical_pts(
    const IDPose *cur_pose, std::vector<IDPtIntensity *> &nearby_pts,
    std::vector<std::pair<Eigen::Vector3d, float>> &pts_sphere,
    double lidarRange, double voxelAngle) {
  std::vector<IDPtIntensity *> new_nearby_pts, pts_sphere_raw;
  for (auto &p : nearby_pts) {
    Eigen::Vector4d p_g(p->pt(0), p->pt(1), p->pt(2), 1.0);
    Eigen::Vector3d p_l = cur_pose->w2c * p_g;

    if (p_l.norm() < lidarRange) {
      pts_sphere_raw.push_back(
          new IDPtIntensity(p->incoming_id, p_l, p->intensity));
    } else if (p_l(2) < 0 ||
               (cur_pose->incoming_id - p->incoming_id) > PTS_HIST)
      continue;

    new_nearby_pts.push_back(p);
  }
  nearby_pts = new_nearby_pts; // update nearby pts

  // filter points
  // filterPoints(pts_sphere_raw, {voxelAngle,voxel_size,voxelAngle},
  // pts_sphere); filterPointsPolar(pts_sphere_raw, lidarRange, {1,1,0.1},
  // pts_sphere);
  filterPointsPolar(pts_sphere_raw, lidarRange, {voxelAngle, voxelAngle, 1},
                    pts_sphere, false);

  printf("\rFrame count: %d, Pts (Total: %lu, Sphere: %lu, Filtered: %lu)",
         cur_pose->incoming_id, nearby_pts.size(), pts_sphere_raw.size(),
         pts_sphere.size());
  fflush(stdout);
}

void pts_preprocess(std::string &poses_history_file,
                    std::string &pts_history_file,
                    std::string &incoming_id_file, double lidarRange,
                    double voxelAngle,
                    std::vector<std::vector<std::pair<Eigen::Vector3d, float>>>
                        &pts_sphere_vec) {
  // read data
  std::vector<IDPose *> poses_history;
  std::vector<IDPtIntensity *> pts_history;
  read_poses_pts(poses_history_file, pts_history_file, poses_history,
                 pts_history);

  std::ofstream incoming_id_file_stream;
  incoming_id_file_stream.open(incoming_id_file);
  std::vector<IDPtIntensity *> nearby_pts;
  std::vector<std::pair<Eigen::Vector3d, float>> pts_sphere;
  int pts_idx(0), frame_from_reset(0), pts_count(0);
  std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
  for (auto &cur_pose : poses_history) {
    // reset
    if (cur_pose->w2c.col(3).norm() < 1.0) {
      printf("\nReset at id: %d\n", cur_pose->incoming_id);
      frame_from_reset = 0;
      nearby_pts.clear();
    }

    // retrive new pts
    while (pts_idx < pts_history.size() &&
           std::abs(cur_pose->incoming_id - pts_history[pts_idx]->incoming_id) <
               PTS_HIST) {
      nearby_pts.push_back(pts_history[pts_idx]);
      pts_idx++;
    }

    // accumulate enough pts
    if (frame_from_reset < INIT_FRAME) {
      frame_from_reset++;
      continue;
    }

    // get sphere pts
    pts_sphere.clear();
    get_spherical_pts(cur_pose, nearby_pts, pts_sphere, lidarRange, voxelAngle);
    pts_sphere_vec.push_back(pts_sphere);
    pts_count += pts_sphere.size();

    incoming_id_file_stream << cur_pose->incoming_id << std::endl;
  }
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  float ttOpt =
      std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0)
          .count();
  std::cout << std::endl
            << "get_spherical_pts average time: "
            << 1000.0 * ttOpt / pts_sphere_vec.size();
  std::cout << "ms average points: " << float(pts_count) / pts_sphere_vec.size()
            << std::endl;
  incoming_id_file_stream.close();

  for (auto &ps : poses_history)
    delete ps;
  for (auto &pt : pts_history)
    delete pt;
}
