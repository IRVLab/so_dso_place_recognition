#ifndef PTS_PREPROCESS_H
#define PTS_PREPROCESS_H

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "PosesPts.h"

#define INIT_FRAME 30
#define RES_GRID 30
#define RES_POLAR 1.0 / 180.0 * M_PI

inline void read_poses_pts(const std::string &poses_history_file_name,
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

inline void
filterPoints(std::vector<IDPtIntensity *> &inPtsI, double lidar_range,
             std::vector<double> resolution,
             std::vector<std::pair<Eigen::Vector3d, float>> &outPtsI) {
  std::vector<double> res_xyz = {lidar_range / resolution[0],
                                 lidar_range / resolution[1],
                                 lidar_range / resolution[2]};
  std::vector<double> steps{1.0 / res_xyz[0], 1.0 / res_xyz[1],
                            1.0 / res_xyz[2]};
  std::vector<int> voxel_size{
      static_cast<int>(floor(2 * lidar_range * steps[0]) + 1),
      static_cast<int>(floor(2 * lidar_range * steps[1]) + 1),
      static_cast<int>(floor(2 * lidar_range * steps[2]) + 1)};
  std::vector<int> loc_step{1, voxel_size[0], voxel_size[0] * voxel_size[1]};

  // get/filter spherical points
  std::unordered_map<int, std::pair<int, Eigen::Vector3d>> loc2idx_pt;
  for (size_t idx = 0; idx < inPtsI.size(); idx++) {
    auto pt = inPtsI[idx]->pt;

    // voxel indices
    int xi = static_cast<int>(floor((pt(0) + lidar_range) * steps[0]));
    int yi = static_cast<int>(floor((pt(1) + lidar_range) * steps[1]));
    int zi = static_cast<int>(floor((pt(2) + lidar_range) * steps[2]));
    int loc = xi * loc_step[0] + yi * loc_step[1] + zi * loc_step[2];

    // store the highest points
    if (loc2idx_pt.find(loc) == loc2idx_pt.end() ||
        -loc2idx_pt[loc].second(1) < -pt(1)) {
      loc2idx_pt[loc] = {idx, pt};
    }
  }

  // output useful points
  for (auto &l_ip : loc2idx_pt) {
    int idx = l_ip.second.first;
    Eigen::Vector3d pt = l_ip.second.second;
    outPtsI.push_back({pt, inPtsI[idx]->intensity});
  }

  for (int idx = 0; idx < inPtsI.size(); idx++) {
    delete inPtsI[idx];
  }
}

inline void
filterPointsPolar(std::vector<IDPtIntensity *> &inPtsI,
                  std::vector<double> resolution,
                  std::vector<std::pair<Eigen::Vector3d, float>> &outPtsI) {
  // Compute the bounding voxel sizes
  double azi_res_inv = 1.0 / resolution[0];
  double ele_res_inv = 1.0 / resolution[1];
  int azi_bins = static_cast<int>(floor(2 * M_PI * azi_res_inv) + 1);

  // Go over all points and insert them into the right leaf
  std::unordered_map<int, std::pair<int, Eigen::Vector3d>> loc2idx_pt;
  for (int idx = 0; idx < inPtsI.size(); idx++) {
    auto pt = inPtsI[idx]->pt;
    double xz = sqrt(pt(0) * pt(0) + pt(2) * pt(2));
    int azi =
        static_cast<int>(floor((atan2(pt(2), pt(0)) + M_PI) * azi_res_inv));
    int ele =
        static_cast<int>(floor((atan2(pt(1), xz) + M_PI / 2) * ele_res_inv));
    int loc = azi + ele * azi_bins;

    // store the closest points
    if (loc2idx_pt.find(loc) == loc2idx_pt.end() ||
        loc2idx_pt[loc].second.norm() > pt.norm()) {
      loc2idx_pt[loc] = {idx, pt};
    }
  }

  // output useful points
  for (auto &l_ip : loc2idx_pt) {
    int idx = l_ip.second.first;
    Eigen::Vector3d pt = l_ip.second.second;
    outPtsI.push_back({pt, inPtsI[idx]->intensity});
  }

  for (int idx = 0; idx < inPtsI.size(); idx++) {
    delete inPtsI[idx];
  }
}

inline void generate_spherical_points(
    const IDPose *cur_pose, std::vector<IDPtIntensity *> &nearby_pts,
    std::vector<std::pair<Eigen::Vector3d, float>> &pts_spherical,
    double lidarRange, bool polar_filter) {
  std::vector<IDPtIntensity *> new_nearby_pts, pts_spherical_raw;
  for (auto &p : nearby_pts) {
    Eigen::Vector4d p_g(p->pt(0), p->pt(1), p->pt(2), 1.0);
    Eigen::Vector3d p_l = cur_pose->w2c * p_g;

    if (p_l.norm() < lidarRange) {
      pts_spherical_raw.push_back(
          new IDPtIntensity(p->incoming_id, p_l, p->intensity));
      new_nearby_pts.push_back(p);
    }
  }

  // filter points
  if (polar_filter) {
    filterPointsPolar(pts_spherical_raw, {RES_POLAR, RES_POLAR, RES_POLAR},
                      pts_spherical);
  } else {
    filterPoints(pts_spherical_raw, lidarRange,
                 {RES_GRID, 2 * RES_GRID, RES_GRID}, pts_spherical);
  }

  printf("\rFrame count: %d, Pts (Total: %lu, Sphere: %lu, Filtered: %lu)",
         cur_pose->incoming_id, nearby_pts.size(), pts_spherical_raw.size(),
         pts_spherical.size());
  fflush(stdout);

  // update nearby pts
  nearby_pts = new_nearby_pts;
}

inline void
pts_preprocess(std::string &poses_history_file, std::string &pts_history_file,
               std::string &incoming_id_file, double lidarRange,
               std::vector<std::vector<std::pair<Eigen::Vector3d, float>>>
                   &pts_spherical_vec,
               bool polar_filter) {
  // read data
  std::vector<IDPose *> poses_history;
  std::vector<IDPtIntensity *> pts_history;
  read_poses_pts(poses_history_file, pts_history_file, poses_history,
                 pts_history);

  std::ofstream incoming_id_file_stream;
  incoming_id_file_stream.open(incoming_id_file);
  std::vector<IDPtIntensity *> nearby_pts;
  std::vector<std::pair<Eigen::Vector3d, float>> pts_spherical;
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
           pts_history[pts_idx]->incoming_id <= cur_pose->incoming_id) {
      nearby_pts.push_back(pts_history[pts_idx]);
      pts_idx++;
    }

    // accumulate enough pts
    if (frame_from_reset < INIT_FRAME) {
      frame_from_reset++;
      continue;
    }

    // get sphere pts
    pts_spherical.clear();
    generate_spherical_points(cur_pose, nearby_pts, pts_spherical, lidarRange,
                              polar_filter);
    pts_spherical_vec.push_back(pts_spherical);
    pts_count += pts_spherical.size();

    incoming_id_file_stream << cur_pose->incoming_id << std::endl;
  }
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  float ttOpt =
      std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0)
          .count();
  std::cout << std::endl
            << "generate_spherical_points average time: "
            << 1000.0 * ttOpt / pts_spherical_vec.size();
  std::cout << "ms average points: "
            << float(pts_count) / pts_spherical_vec.size() << std::endl;
  incoming_id_file_stream.close();

  for (auto &ps : poses_history)
    delete ps;
  for (auto &pt : pts_history)
    delete pt;
}

#endif