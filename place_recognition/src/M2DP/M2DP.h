#pragma once
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <vector>

class M2DP {
public:
  M2DP();
  M2DP(int t, int r, int p, int q);

  unsigned int getSignatureSize();

  void
  getSignature(const std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr,
               Eigen::VectorXd &count_output, Eigen::VectorXd &color_output,
               double maxRho = -1.0);

  void PCARotationInvariant(
      const std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr_in,
      std::vector<std::vector<std::pair<Eigen::Vector3d, float>>> &pts_clr_out);

private:
  int numT;
  int numR;
  int numP;
  int numQ;

  // Projection matrix
  void init();
  Eigen::MatrixXd xProj_mat;
  Eigen::MatrixXd yProj_mat;
};
