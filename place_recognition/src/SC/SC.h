#pragma once
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <vector>

class SC {
public:
  SC();
  SC(int s, int r);

  unsigned int getSignatureSize();

  void
  getSignature(const std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr,
               Eigen::VectorXd &count_output, Eigen::VectorXd &color_output,
               double maxRho = -1.0);

  void PCARotationInvariant(
      const std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr_in,
      std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr_out);

private:
  int numS;
  int numR;
};
