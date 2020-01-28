#include "SC.h"
#include "place_recognition/src/utils/pts_align.h"
#include <cmath>

SC::SC(double max_rho) {
  S_res_inv = numS / (2.0 * M_PI);
  R_res_inv = numR / max_rho;
}

unsigned int SC::getSignatureSize() { return numS * numR; }

void SC::getSignature(
    const std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr_raw,
    Eigen::VectorXd &structure_output, Eigen::VectorXd &intensity_output) {
  // align points by PCA
  std::vector<std::pair<Eigen::Vector3d, float>> pts_clr;
  align_points_PCA(pts_clr_raw, pts_clr);

  // signature matrix A
  Eigen::VectorXd pts_count =
      Eigen::VectorXd::Zero(getSignatureSize()); // pts count
  Eigen::VectorXd height_lowest =
      Eigen::VectorXd::Zero(getSignatureSize()); // height lowest
  Eigen::VectorXd height_highest =
      Eigen::VectorXd::Zero(getSignatureSize()); // height highest
  Eigen::VectorXd pts_intensity =
      Eigen::VectorXd::Zero(getSignatureSize()); // intensity

  for (int i = 0; i < pts_clr.size(); i++) // loop on pts
  {
    // projection to polar coordinate
    // After PCA, x: up; y: left; z:back
    double yp = pts_clr[i].first(1);
    double zp = pts_clr[i].first(2);

    // get projection bin w.r.t. theta and rho
    int si = static_cast<int>(floor((atan2(zp, yp) + M_PI) * S_res_inv));
    int ri = static_cast<int>(floor(std::sqrt(yp * yp + zp * zp) * R_res_inv));
    int idx = si * numR + ri;

    // PCA moves the points
    if (idx >= getSignatureSize()) {
      continue;
    }

    if (pts_count(idx) == 0) {
      pts_intensity(idx) = pts_clr[i].second;
      height_lowest(idx) = pts_clr[i].first(0);
      height_highest(idx) = pts_clr[i].first(0);
    } else {
      pts_intensity(idx) += double(pts_clr[i].second);
      height_lowest(idx) = std::min(height_lowest(idx), pts_clr[i].first(0));
      height_highest(idx) = std::max(height_highest(idx), pts_clr[i].first(0));
    }

    pts_count(idx)++;
  }

  // average intensity
  float ave_intensity = 0;
  for (int i = 0; i < pts_clr.size(); i++) {
    ave_intensity += pts_clr[i].second;
  }
  ave_intensity = ave_intensity / pts_clr.size();

  // binarize average intensity for each bin
  for (int i = 0; i < getSignatureSize(); i++) {
    if (pts_count(i)) {
      pts_intensity(i) = pts_intensity(i) / pts_count(i);
      pts_intensity(i) = pts_intensity(i) > ave_intensity ? 1 : 0;
    }
  }

  structure_output = height_highest - height_lowest; // height difference
  intensity_output = pts_intensity;
}
