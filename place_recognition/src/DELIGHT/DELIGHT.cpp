#include "DELIGHT.h"
#include "place_recognition/src/utils/pts_align.h"

unsigned int DELIGHT::getSignatureSize() { return BINS; }

void DELIGHT::getSignature(
    const std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr_raw,
    Eigen::MatrixXd &output) {
  // align points by PCA
  std::vector<std::pair<Eigen::Vector3d, float>> pts_clr;
  align_points_PCA(pts_clr_raw, pts_clr);

  output = Eigen::MatrixXd::Zero(16, BINS);
  for (auto &p : pts_clr) {
    float x = p.first(0);
    float y = p.first(1);
    float z = p.first(2);
    float d = p.first.norm();
    float clr = p.second;

    int hist = 8 * (d > RADIUS) + 4 * (z > 0) + 2 * (y > 0) + 1 * (x > 0);
    output(hist, int(clr))++;
  }
}
