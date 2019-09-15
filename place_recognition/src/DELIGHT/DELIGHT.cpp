#include "DELIGHT.h"

#define PI 3.1415926

DELIGHT::DELIGHT() {
  inner_radius = 10;
  bins = 256;
}

DELIGHT::DELIGHT(float _inner_radius, int _bins)
    : inner_radius(_inner_radius), bins(_bins) {}

unsigned int DELIGHT::getSignatureSize() { return bins; }

void DELIGHT::getSignature(
    const std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr,
    Eigen::MatrixXd &output) {
  output = Eigen::MatrixXd::Zero(16, bins);
  for (auto &p : pts_clr) {
    float x = p.first(0);
    float y = p.first(1);
    float z = p.first(2);
    float d = p.first.norm();
    float clr = p.second;

    int hist = 8 * (d > inner_radius) + 4 * (z > 0) + 2 * (y > 0) + 1 * (x > 0);
    output(hist, int(clr))++;
  }
}
