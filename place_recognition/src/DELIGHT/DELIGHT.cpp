#include "DELIGHT.h"

#define PI 3.1415926

DELIGHT::DELIGHT() {
  r1 = 40;
  r2 = 10;
  b = 256;
}

DELIGHT::DELIGHT(float _r1, float _r2, int _b) : r1(_r1), r2(_r2), b(_b) {}

unsigned int DELIGHT::getSignatureSize() { return b; }

void DELIGHT::getSignature(
    const std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr,
    Eigen::MatrixXd &output) {
  output = Eigen::MatrixXd::Zero(16, b);
  for (auto &p : pts_clr) {
    float x = p.first(0);
    float y = p.first(1);
    float z = p.first(2);
    float d = p.first.norm();
    float clr = p.second;

    // std::cout<<"clr "<<clr<<std::endl;
    int bi = 8 * (d > r2) + 4 * (z > 0) + 2 * (y > 0) + 1 * (x > 0);
    output(bi, int(clr))++;
  }
}

void DELIGHT::PCARotationInvariant(
    const std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr_in,
    std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr_out) {
  double mx(0), my(0), mz(0);
  for (auto &pc : pts_clr_in) {
    mx += pc.first(0);
    my += pc.first(1);
    mz += pc.first(2);
  }
  mx /= pts_clr_in.size();
  my /= pts_clr_in.size();
  mz /= pts_clr_in.size();

  // normalize pts
  Eigen::MatrixXd pts_mat(pts_clr_in.size(), 3);
  for (int i = 0; i < pts_clr_in.size(); i++) {
    pts_mat(i, 0) = pts_clr_in[i].first(0) - mx;
    pts_mat(i, 1) = pts_clr_in[i].first(1) - my;
    pts_mat(i, 2) = pts_clr_in[i].first(2) - mz;
  }

  // PCA
  Eigen::MatrixXd cov = pts_mat.transpose() * pts_mat;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(cov);
  Eigen::VectorXd v0 = es.eigenvectors().col(0);
  Eigen::VectorXd v1 = es.eigenvectors().col(1);
  Eigen::VectorXd v2 = es.eigenvectors().col(2);
  // std::cout<<v0<<std::endl;
  // std::cout<<v1<<std::endl;
  // std::cout<<v2<<std::endl;

  // rotate pts
  Eigen::VectorXd nx = pts_mat * v0;
  Eigen::VectorXd ny = pts_mat * v1;
  Eigen::VectorXd nz = pts_mat * v2;

  pts_clr_out.clear();
  for (int i = 0; i < pts_clr_in.size(); i++) {
    pts_clr_out.push_back(
        {Eigen::Vector3d(nx(i), ny(i), nz(i)), pts_clr_in[i].second});
  }
  // std::cout<<pts_rot[0].first<<std::endl;
  // std::cout<<"------------------------"<<std::endl;
}
