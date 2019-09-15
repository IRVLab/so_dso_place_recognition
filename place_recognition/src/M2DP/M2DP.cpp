#include "M2DP.h"
#include <cmath>

M2DP::M2DP() {
  numT = 16;
  numR = 8;
  numP = 4;
  numQ = 16;

  init();
}

M2DP::M2DP(int t, int r, int p, int q) : numT(t), numR(r), numP(p), numQ(q) {
  init();
}

void M2DP::init() {
  xProj_mat = Eigen::MatrixXd::Zero(3, numP * numQ);
  yProj_mat = Eigen::MatrixXd::Zero(3, numP * numQ);

  // loop on azimuth
  for (int p = 0; p < numP; p++) {
    float azm = -M_PI / 2.0 + (M_PI / numP) * p;

    // loop on elevation
    for (int q = 0; q < numQ; q++) {
      float elv = (M_PI / 2.0 / numQ) * q;

      // normal vector of current 2D plane
      Eigen::Vector3d vecN(std::cos(elv) * std::cos(azm),
                           std::cos(elv) * std::sin(azm), std::sin(elv));

      // x-axis projection
      Eigen::Vector3d xAxis(1, 0, 0);
      Eigen::Vector3d xProj = xAxis - (xAxis.transpose() * vecN) * vecN;

      // y-axis by cross production
      Eigen::Vector3d yProj = vecN.cross(xProj);

      xProj_mat.col(p * numQ + q) = xProj;
      yProj_mat.col(p * numQ + q) = yProj;
    }
  }
}

unsigned int M2DP::getSignatureSize() { return numT * numR + numP * numQ; }

void M2DP::getSignature(
    const std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr,
    Eigen::VectorXd &count_output, Eigen::VectorXd &intensity_coutput,
    double max_rho) {
  if (max_rho < 0) {
    for (auto &pc : pts_clr) {
      double curRho = pc.first.norm();
      if (max_rho < curRho)
        max_rho = curRho;
    }
  }

  // signature matrix A
  Eigen::MatrixXd pts_count =
      Eigen::MatrixXd::Zero(numP * numQ, numT * numR); // pts count
  Eigen::MatrixXd pts_intensity =
      Eigen::MatrixXd::Zero(numP * numQ, numT * numR); // intensity

  for (int p = 0; p < numP; p++) // loop on azimuth
  {
    for (int q = 0; q < numQ; q++) // loop on elevation
    {
      Eigen::Vector3d xProj = xProj_mat.col(p * numQ + q);
      Eigen::Vector3d yProj = yProj_mat.col(p * numQ + q);
      for (int i = 0; i < pts_clr.size(); i++) // loop on pts
      {
        // projection to polar coordinate
        double xp = xProj.dot(pts_clr[i].first);
        double yp = yProj.dot(pts_clr[i].first);
        double rho = std::sqrt(xp * xp + yp * yp);
        double theta = std::atan2(yp, xp);
        while (theta < 0)
          theta += 2.0 * M_PI;
        while (theta >= 2.0 * M_PI)
          theta -= 2.0 * M_PI;

        // get projection bin w.r.t. theta and rho
        int ti = theta / (2.0 * M_PI) * numT;
        if (ti == numT)
          ti = 0;
        int ri = rho / max_rho * numR;
        if (ri >= numR)
          continue;

        pts_count(p * numQ + q, ri * numT + ti)++;
        pts_intensity(p * numQ + q, ri * numT + ti) += pts_clr[i].second;
      }
    }
  }

  // average intensity
  float ave_intensity = 0;
  for (int i = 0; i < pts_clr.size(); i++) {
    ave_intensity += pts_clr[i].second;
  }
  ave_intensity = ave_intensity / pts_clr.size();

  // binarize average intensity for each bin
  for (int i = 0; i < numP * numQ; i++) {
    for (int j = 0; j < numR * numT; j++) {
      if (pts_count(i, j)) {
        pts_intensity(i, j) = pts_intensity(i, j) / pts_count(i, j);
        pts_intensity(i, j) = pts_intensity(i, j) > ave_intensity ? 1 : 0;
      }
    }
  }

  // SVD
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_pts_count(
      pts_count, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd U1_pts_count = svd_pts_count.matrixU().col(0);
  Eigen::VectorXd V1_pts_count = svd_pts_count.matrixV().col(0);
  count_output = Eigen::VectorXd(U1_pts_count.rows() + V1_pts_count.rows());

  Eigen::JacobiSVD<Eigen::MatrixXd> svd_intensity(
      pts_intensity, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd U1_intensity = svd_intensity.matrixU().col(0);
  Eigen::VectorXd V1_intensity = svd_intensity.matrixV().col(0);
  intensity_coutput =
      Eigen::VectorXd(U1_intensity.rows() + V1_intensity.rows());

  count_output << U1_pts_count, V1_pts_count;
  intensity_coutput << U1_intensity, V1_intensity;
}
