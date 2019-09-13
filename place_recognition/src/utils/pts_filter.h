#include <Eigen/Core>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

using namespace std;

struct Point_Voxel {
  int idx;
  int voxel;
  Point_Voxel(int arg_idx, int arg_voxel) {
    idx = arg_idx;
    voxel = arg_voxel;
  }
  bool operator<(const Point_Voxel &rhs) const { return (voxel < rhs.voxel); }
};

template <class T> struct vec {
  T x, y, z;
  float i;

  vec<T>() {}

  vec<T>(T arg_x, T arg_y, T arg_z) {
    x = arg_x;
    y = arg_y;
    z = arg_z;
  }

  vec<T>(T arg_x, T arg_y, T arg_z, float arg_i) {
    x = arg_x;
    y = arg_y;
    z = arg_z;
    i = arg_i;
  }

  vec<T> &operator=(const vec<T> &rhs) {
    x = rhs.x;
    y = rhs.y;
    z = rhs.z;
    i = rhs.i;
    return *this;
  }

  vec<T>(const vec<T> &rhs) {
    x = rhs.x;
    y = rhs.y;
    z = rhs.z;
    i = rhs.i;
  }

  vec<T> &operator+=(const vec<T> &rhs) {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    i += rhs.i;
    return *this;
  }
};

void getMinMax(vector<vec<double>> &inCloud, vec<double> &minp,
               vec<double> &maxp) {
  for (int i = 0; i < inCloud.size(); i++) {
    minp.x = std::min(minp.x, inCloud[i].x);
    minp.y = std::min(minp.y, inCloud[i].y);
    minp.z = std::min(minp.z, inCloud[i].z);

    maxp.x = std::max(maxp.x, inCloud[i].x);
    maxp.y = std::max(maxp.y, inCloud[i].y);
    maxp.z = std::max(maxp.z, inCloud[i].z);
  }
}

void filterPoints(vector<IDPtIntensity *> &inPtsI, vector<double> leafSizeVec,
                  vector<pair<Eigen::Vector3d, float>> &outPtsI) {
  vector<vec<double>> inCloud;
  for (auto &p : inPtsI)
    inCloud.push_back(vec<double>(p->pt(0), p->pt(1), p->pt(2), p->intensity));

  // Compute minimum and maximum point values
  vec<double> minp(DBL_MAX, DBL_MAX, DBL_MAX),
      maxp(-DBL_MAX, -DBL_MAX, -DBL_MAX);
  getMinMax(inCloud, minp, maxp);

  // //Compute bounding voxel sizes
  vec<double> inv_leafSize(1.0 / leafSizeVec[0], 1.0 / leafSizeVec[1],
                           1.0 / leafSizeVec[2]);

  // Compute the minimum and maximum bounding voxel values
  vec<int> minb(static_cast<int>(floor(minp.x * inv_leafSize.x)),
                static_cast<int>(floor(minp.y * inv_leafSize.y)),
                static_cast<int>(floor(minp.z * inv_leafSize.z)));

  vec<int> maxb(static_cast<int>(floor(maxp.x * inv_leafSize.x)),
                static_cast<int>(floor(maxp.y * inv_leafSize.y)),
                static_cast<int>(floor(maxp.z * inv_leafSize.z)));

  vec<int> divb(maxb.x - minb.x + 1, maxb.y - minb.y + 1, maxb.z - minb.z + 1);
  vec<int> divb_mul(1, divb.x, divb.x * divb.y);

  // Go over all points and insert them into the right leaf
  vector<Point_Voxel> indices;
  for (int i = 0; i < inCloud.size(); i++) {
    int ijk0 = static_cast<int>(floor(inCloud[i].x * inv_leafSize.x) - minb.x);
    int ijk1 = static_cast<int>(floor(inCloud[i].y * inv_leafSize.y) - minb.y);
    int ijk2 = static_cast<int>(floor(inCloud[i].z * inv_leafSize.z) - minb.z);
    int idx = ijk0 * divb_mul.x + ijk1 * divb_mul.y + ijk2 * divb_mul.z;
    indices.push_back(Point_Voxel(i, idx));
  }
  sort(indices.begin(), indices.end(), less<Point_Voxel>());
  for (int cp = 0; cp < inCloud.size();) {
    vec<double> centroid(inCloud[indices[cp].idx].x, inCloud[indices[cp].idx].y,
                         inCloud[indices[cp].idx].z,
                         inCloud[indices[cp].idx].i);
    int i = cp + 1;
    while (i < inCloud.size() && indices[cp].voxel == indices[i].voxel) {
      centroid += inCloud[indices[i].idx];
      ++i;
    }
    centroid.x /= static_cast<double>(i - cp);
    centroid.y /= static_cast<double>(i - cp);
    centroid.z /= static_cast<double>(i - cp);
    centroid.i /= static_cast<double>(i - cp);

    Eigen::Vector3d pts(centroid.x, centroid.y, centroid.z);
    outPtsI.push_back({pts, centroid.i});
    cp = i;
  }
}

void filterPointsPolar(vector<IDPtIntensity *> &inPtsI, double range,
                       vector<double> azi_ele_rho,
                       vector<pair<Eigen::Vector3d, float>> &outPtsI,
                       bool seeThrough = true) {
  vector<vec<double>> inCloud;
  for (auto &p : inPtsI)
    inCloud.push_back(vec<double>(p->pt(0), p->pt(1), p->pt(2), p->intensity));

  // //Compute bounding voxel sizes
  azi_ele_rho[0] =
      azi_ele_rho[0] / 180 * M_PI; // azimuth is originally represented in angle
  azi_ele_rho[1] = azi_ele_rho[1] / 180 *
                   M_PI; // elevation is originally represented in angle
  vector<double> inv_leafSize{1.0 / azi_ele_rho[0], 1.0 / azi_ele_rho[1],
                              1.0 / azi_ele_rho[2]};

  // Compute the bounding voxel sizes
  vector<int> divb{static_cast<int>(floor(2 * M_PI * inv_leafSize[0]) + 1),
                   static_cast<int>(floor(M_PI * inv_leafSize[1]) + 1),
                   static_cast<int>(floor(range * inv_leafSize[2]) + 1)};
  vector<int> divb_mul{1, divb[0], divb[0] * divb[1]};
  if (!seeThrough)
    divb_mul[2] = 0;

  // Go over all points and insert them into the right leaf
  vector<Point_Voxel> indices;
  for (int i = 0; i < inCloud.size(); i++) {
    double xz = sqrt(inCloud[i].x * inCloud[i].x + inCloud[i].z * inCloud[i].z);
    int ijk0 = static_cast<int>(
        floor((atan2(inCloud[i].z, inCloud[i].x) + M_PI) * inv_leafSize[0]));
    int ijk1 = static_cast<int>(
        floor((atan2(inCloud[i].y, xz) + M_PI / 2) * inv_leafSize[1]));
    int ijk2 = static_cast<int>(
        floor(sqrt(xz * xz + inCloud[i].y * inCloud[i].y) * inv_leafSize[2]));
    int idx = ijk0 * divb_mul[0] + ijk1 * divb_mul[1] + ijk2 * divb_mul[2];
    indices.push_back(Point_Voxel(i, idx));
  }
  sort(indices.begin(), indices.end(), less<Point_Voxel>());
  if (!seeThrough) {
    for (int cp = 0; cp < inCloud.size();) {
      int i = cp + 1;
      double min_rho = inCloud[indices[cp].idx].z;
      int min_i = cp;
      while (i < inCloud.size() && indices[cp].voxel == indices[i].voxel) {
        if (min_rho > inCloud[indices[i].idx].z) {
          min_rho = inCloud[indices[i].idx].z;
          min_i = i;
        }
        ++i;
      }
      vec<double> closest(
          inCloud[indices[min_i].idx].x, inCloud[indices[min_i].idx].y,
          inCloud[indices[min_i].idx].z, inCloud[indices[min_i].idx].i);

      Eigen::Vector3d pts(closest.x, closest.y, closest.z);
      outPtsI.push_back({pts, closest.i});
      cp = i;
    }
  } else {
    for (int cp = 0; cp < inCloud.size();) {
      vec<double> centroid(
          inCloud[indices[cp].idx].x, inCloud[indices[cp].idx].y,
          inCloud[indices[cp].idx].z, inCloud[indices[cp].idx].i);
      int i = cp + 1;
      while (i < inCloud.size() && indices[cp].voxel == indices[i].voxel) {
        centroid += inCloud[indices[i].idx];
        ++i;
      }
      centroid.x /= static_cast<double>(i - cp);
      centroid.y /= static_cast<double>(i - cp);
      centroid.z /= static_cast<double>(i - cp);
      centroid.i /= static_cast<double>(i - cp);

      Eigen::Vector3d pts(centroid.x, centroid.y, centroid.z);
      outPtsI.push_back({pts, centroid.i});
      cp = i;
    }
  }
}
