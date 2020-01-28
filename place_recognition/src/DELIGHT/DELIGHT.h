#pragma once
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <math.h>
#include <vector>

#define RADIUS 10.0
#define BINS 256

class DELIGHT {
public:
  unsigned int getSignatureSize();

  void getSignature(
      const std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr_raw,
      Eigen::MatrixXd &output);
};
