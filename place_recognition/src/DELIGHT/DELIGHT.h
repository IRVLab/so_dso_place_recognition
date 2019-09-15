#pragma once
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <math.h>
#include <vector>

class DELIGHT {
public:
  DELIGHT();
  DELIGHT(float _inner_radius, int _bins);

  unsigned int getSignatureSize();

  void
  getSignature(const std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr,
               Eigen::MatrixXd &output);

private:
  float inner_radius;
  int bins;
};
