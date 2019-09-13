#pragma once
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <math.h>
#include <vector>

class DELIGHT {
public:
  DELIGHT();
  DELIGHT(float r1, float r2, int b);

  unsigned int getSignatureSize();

  void
  getSignature(const std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr,
               Eigen::MatrixXd &output);

  void PCARotationInvariant(
      const std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr_in,
      std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr_out);

private:
  float r1;
  float r2;
  int b;
};
