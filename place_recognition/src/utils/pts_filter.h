#include <Eigen/Core>
#include <algorithm>
#include <unordered_map>
#include <vector>

using namespace std;

void filterPoints(vector<IDPtIntensity *> &inPtsI, double lidar_range,
                  vector<double> resolution,
                  vector<pair<Eigen::Vector3d, float>> &outPtsI) {
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
}

void filterPointsPolar(vector<IDPtIntensity *> &inPtsI,
                       vector<double> resolution,
                       vector<pair<Eigen::Vector3d, float>> &outPtsI) {
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

    // store the highest points
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
}
