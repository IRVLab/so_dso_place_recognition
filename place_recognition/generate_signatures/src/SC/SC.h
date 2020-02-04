#pragma once
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <vector>

#define numS 60
#define numR 20

class SC {
public:
  SC(double max_rho);

  unsigned int getSignatureSize();

  void getSignature(
      const std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr_raw,
      Eigen::VectorXd &structure_output, Eigen::VectorXd &intensity_output);

private:
  double S_res_inv;
  double R_res_inv;
};
