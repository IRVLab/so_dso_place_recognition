#pragma once
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <vector>

#define numS 16 // theta
#define numR 8  // rho
#define numP 4
#define numQ 16

class M2DP {
public:
  M2DP(double max_rho);

  unsigned int getSignatureSize();

  void getSignature(
      const std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr_raw,
      Eigen::VectorXd &count_output, Eigen::VectorXd &intensity_coutput);

private:
  double S_res_inv;
  double R_res_inv;

  // Projection matrix
  void init();
  Eigen::MatrixXd xProj_mat;
  Eigen::MatrixXd yProj_mat;
};
