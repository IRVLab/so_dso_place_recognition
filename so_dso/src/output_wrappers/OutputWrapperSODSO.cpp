#include "OutputWrapperSODSO.h"
#include <algorithm>

bool PoseCompare(const IDPose *l, const IDPose *r) {
  return l->incoming_id < r->incoming_id;
}

bool PtCompare(const IDPtIntensity *l, const IDPtIntensity *r) {
  return l->incoming_id < r->incoming_id;
}

namespace dso {
namespace IOWrap {

OutputWrapperSODSO::~OutputWrapperSODSO() {
  std::ofstream poses_history_file, pts_history_file;
  poses_history_file.open("poses_history_file.txt");
  pts_history_file.open("pts_history_file.txt");

  // sort by id
  std::sort(poses_history.begin(), poses_history.end(), PoseCompare);
  std::sort(pts_history.begin(), pts_history.end(), PtCompare);

  for (auto pose : poses_history) {
    poses_history_file << *pose;
    delete pose;
  }
  for (auto pt : pts_history) {
    pts_history_file << *pt;
    delete pt;
  }

  pts_history_file.close();
  poses_history_file.close();
}

void OutputWrapperSODSO::publishKeyframes(std::vector<FrameHessian *> &frames,
                                          bool final, CalibHessian *HCalib) {
  if (!final)
    return;
  float fx = HCalib->fxl();
  float fy = HCalib->fyl();
  float cx = HCalib->cxl();
  float cy = HCalib->cyl();
  for (FrameHessian *fh : frames) {
    for (PointHessian *p : fh->pointHessiansMarginalized) {
      float ave_color = 0;
      for (int i = 0; i < patternNum; i++)
        ave_color += p->color[i];
      ave_color /= patternNum;

      Eigen::Vector4d p_l((p->u - cx) / fx / p->idepth_scaled,
                          (p->v - cy) / fy / p->idepth_scaled,
                          1 / p->idepth_scaled, 1);
      Eigen::Vector3d p_g = fh->shell->camToWorld.matrix3x4() * p_l;
      pts_history.push_back(
          new IDPtIntensity(fh->shell->incoming_id, p_g, ave_color));
    }
    poses_history.push_back(new IDPose(
        fh->shell->incoming_id, fh->shell->camToWorld.inverse().matrix3x4()));
  }
}
} // namespace IOWrap

} // namespace dso
