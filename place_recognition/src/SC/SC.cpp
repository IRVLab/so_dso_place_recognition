#include "SC.h"
#include <cmath>

SC::SC() {
  numS = 60;
  numR = 20;
}

SC::SC(int s, int r) : numS(s), numR(r) {}

unsigned int SC::getSignatureSize() { return numS * numR; }

void SC::getSignature(
    const std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr,
    Eigen::VectorXd &count_output, Eigen::VectorXd &color_output,
    double maxRho) {
  if (maxRho < 0) {
    for (auto &pc : pts_clr) {
      double curRho = pc.first.norm();
      if (maxRho < curRho)
        maxRho = curRho;
    }
    // std::cout<<"maxRho "<<maxRho<<std::endl;
  }

  // signature matrix A
  Eigen::VectorXd CT = Eigen::VectorXd::Zero(numS * numR); // pts count
  Eigen::VectorXd HL = Eigen::VectorXd::Zero(numS * numR); // height lowest
  Eigen::VectorXd HH = Eigen::VectorXd::Zero(numS * numR); // height highest
  Eigen::VectorXd HD = Eigen::VectorXd::Zero(numS * numR); // height difference
  Eigen::VectorXd IL = Eigen::VectorXd::Zero(numS * numR); // intensity lowest
  Eigen::VectorXd IH = Eigen::VectorXd::Zero(numS * numR); // intensity highest
  Eigen::VectorXd ID =
      Eigen::VectorXd::Zero(numS * numR); // intensity difference
  Eigen::VectorXd IA = Eigen::VectorXd::Zero(numS * numR); // intensity average
  Eigen::VectorXd IB = Eigen::VectorXd::Zero(numS * numR); // intensity binary

  for (int i = 0; i < pts_clr.size(); i++) // loop on pts
  {
    // projection to polar coordinate
    double yp = pts_clr[i].first(1);
    double zp = pts_clr[i].first(2);
    // printf("xp: %f yp: %f zp: %f\n", pts_clr[i].first(0), yp, zp);
    double rho = std::sqrt(yp * yp + zp * zp);
    double theta = std::atan2(zp, yp);
    while (theta < 0)
      theta += 2.0 * M_PI;
    while (theta >= 2.0 * M_PI)
      theta -= 2.0 * M_PI;

    // count points in bins
    // if(rho<1e-9) theta = 0.0;           // TODO

    // get projection bin w.r.t. theta and rho
    int si = theta / (2.0 * M_PI) * numS;
    if (si == numS)
      si = 0;
    int ri = rho / maxRho * numR;
    if (ri >= numR)
      continue; // TODO

    if (CT(si * numR + ri) == 0) {
      IL(si * numR + ri) = pts_clr[i].second;
      IH(si * numR + ri) = pts_clr[i].second;
      HL(si * numR + ri) = pts_clr[i].first(0);
      HH(si * numR + ri) = pts_clr[i].first(0);
    } else {
      IL(si * numR + ri) =
          std::min(IL(si * numR + ri), double(pts_clr[i].second));
      IH(si * numR + ri) =
          std::max(IH(si * numR + ri), double(pts_clr[i].second));
      HL(si * numR + ri) = std::min(HL(si * numR + ri), pts_clr[i].first(0));
      HH(si * numR + ri) = std::max(HH(si * numR + ri), pts_clr[i].first(0));
    }

    IA(si * numR + ri) += pts_clr[i].second;
    CT(si * numR + ri)++;
  }

  float total_int = 0;
  for (int i = 0; i < pts_clr.size(); i++)
    total_int += pts_clr[i].second;
  float ave_int = total_int / pts_clr.size();

  for (int i = 0; i < numS * numR; i++) {
    if (CT(i)) {
      IA(i) /= CT(i);
      IA(i) -= ave_int;
      IB(i) = IA(i) > 0 ? 1 : 0;

      HD(i) = HH(i) - HL(i);
      ID(i) = IH(i) - IL(i);
    }
  }
  // kitti: IL IA HD
  // robotcar: HD IL IA

  count_output = HD;
  color_output = IB;
}

void SC::PCARotationInvariant(
    const std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr_in,
    std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr_out) {
  double mx(0), my(0), mz(0);
  float mc(0);
  for (auto &pc : pts_clr_in) {
    mx += pc.first(0);
    my += pc.first(1);
    mz += pc.first(2);
    mc += pc.second;
  }
  mx /= pts_clr_in.size();
  my /= pts_clr_in.size();
  mz /= pts_clr_in.size();
  mc /= pts_clr_in.size();

  // normalize pts and color
  Eigen::MatrixXd pts_mat(pts_clr_in.size(), 3);
  std::vector<float> color;
  for (int i = 0; i < pts_clr_in.size(); i++) {
    pts_mat(i, 0) = pts_clr_in[i].first(0) - mx;
    pts_mat(i, 1) = pts_clr_in[i].first(1) - my;
    pts_mat(i, 2) = pts_clr_in[i].first(2) - mz;
    color.push_back(pts_clr_in[i].second - mc);
  }

  // PCA
  Eigen::MatrixXd cov = pts_mat.transpose() * pts_mat;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(cov);
  Eigen::VectorXd v0 = es.eigenvectors().col(0);
  Eigen::VectorXd v1 = es.eigenvectors().col(1);
  Eigen::VectorXd v2 = es.eigenvectors().col(2);

  // rotate pts
  Eigen::VectorXd nx = pts_mat * v0;
  Eigen::VectorXd ny = pts_mat * v1;
  Eigen::VectorXd nz = pts_mat * v2;

  pts_clr_out.clear();
  for (int i = 0; i < pts_clr_in.size(); i++) {
    pts_clr_out.push_back({Eigen::Vector3d(nx(i), ny(i), nz(i)), color[i]});
  }
}
