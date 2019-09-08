#include "M2DP.h"
#include <cmath>

M2DP::M2DP() {
  numT = 16;
  numR = 8;
  numP = 4;
  numQ = 16;

  // numT = 4;
  // numR = 4;
  // numP = 4;
  // numQ = 4;

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
  Eigen::MatrixXd CT =
      Eigen::MatrixXd::Zero(numP * numQ, numT * numR); // pts count
  Eigen::MatrixXd CI =
      Eigen::MatrixXd::Zero(numP * numQ, numT * numR); // color intensity
  for (int p = 0; p < numP; p++)                       // loop on azimuth
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

        // count points in bins
        // if(rho<1e-9) theta = 0.0;           // TODO

        // get projection bin w.r.t. theta and rho
        int ti = theta / (2.0 * M_PI) * numT;
        if (ti == numT)
          ti = 0;
        int ri = rho / maxRho * numR;
        if (ri >= numR)
          continue; // TODO

        CT(p * numQ + q, ri * numT + ti)++;
        CI(p * numQ + q, ri * numT + ti) += pts_clr[i].second;
      }
    }
  }

  float total_int = 0;
  for (int i = 0; i < pts_clr.size(); i++)
    total_int += pts_clr[i].second;
  float ave_int = total_int / pts_clr.size();

  for (int i = 0; i < numP * numQ; i++) {
    for (int j = 0; j < numR * numT; j++) {
      if (CT(i, j)) {
        CI(i, j) /= CT(i, j);
        CI(i, j) -= ave_int;
        CI(i, j) = CI(i, j) > 0 ? 1 : 0;
        // CT(i,j) = 1;
      }
      // std::cout<<CI(i,j)<<" ";
    }
    // std::cout<<std::endl;
  }

  // for(int i=0; i<numP*numQ; i++)
  // {
  //   for(int j=0; j<numR*numT; j++)
  //   {
  //     if(CT(i,j))
  //     {
  //       if(CI(i,j)>CI(i, (j%numT)==0?(j+numT-1):(j-1))
  //       && CI(i,j)>CI(i, ((j+1)%numT)==0?(j-numT+1):(j+1))
  //       && CI(i,j)>=CI(i, (j<numT)?j:(j-numT))
  //       && CI(i,j)>=CI(i, ((j+numT)>=numR*numT)?j:j+numT) )
  //       {
  //         CI(i,j)=1;
  //       } else {
  //         CI(i,j)=0;
  //       }
  //     }
  //   }
  // }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd_ct(CT, Eigen::ComputeThinU |
                                                   Eigen::ComputeThinV);
  Eigen::VectorXd U1_ct = svd_ct.matrixU().col(0);
  Eigen::VectorXd V1_ct = svd_ct.matrixV().col(0);
  count_output = Eigen::VectorXd(U1_ct.rows() + V1_ct.rows());
  count_output << U1_ct, V1_ct;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd_ci(CI, Eigen::ComputeThinU |
                                                   Eigen::ComputeThinV);
  Eigen::VectorXd U1_ci = svd_ci.matrixU().col(0);
  Eigen::VectorXd V1_ci = svd_ci.matrixV().col(0);
  color_output = Eigen::VectorXd(U1_ci.rows() + V1_ci.rows());
  color_output << U1_ci, V1_ci;
}

void M2DP::PCARotationInvariant(
    const std::vector<std::pair<Eigen::Vector3d, float>> &pts_clr_in,
    std::vector<std::vector<std::pair<Eigen::Vector3d, float>>> &pts_clr_out) {
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

  // Transformation to reference coordinate
  // Eigen::Matrix3d R;
  // R << v0.transpose(), v1.transpose(), v2.transpose();
  // Eigen::Quaterniond q(R);
  // Eigen::Vector3d t;
  // t << mx, my, mz;
  // t = -R*t;
  // std::cout<<v0.transpose()<<std::endl;
  // std::cout<<v1.transpose()<<std::endl;
  // std::cout<<v2.transpose()<<std::endl;
  // std::cout<<std::endl;
  // std::cout<<R<<std::endl;
  // std::cout<<std::endl;
  // // std::cout<<mx<<" "<<my<<" "<<mz<<std::endl;
  // std::cout<<t.transpose()<<std::endl;
  // std::cout<<std::endl;

  // rotate pts
  Eigen::VectorXd nx = pts_mat * v0;
  Eigen::VectorXd ny = pts_mat * v1;
  Eigen::VectorXd nz = pts_mat * v2;

  pts_clr_out.clear();
  // four possible coordinate
  for (int sign_x = -1; sign_x < 2; sign_x += 2) {
    for (int sign_y = -1; sign_y < 2; sign_y += 2) {
      std::vector<std::pair<Eigen::Vector3d, float>> pts_rot;
      for (int i = 0; i < pts_clr_in.size(); i++) {
        pts_rot.push_back({Eigen::Vector3d(sign_x * nx(i), sign_y * ny(i),
                                           sign_x * sign_y * nz(i)),
                           color[i]});
        // pts_rot.push_back({Eigen::Vector3d(nx(i), ny(i), nz(i)), color[i]});
      }
      // std::cout<<pts_rot[0].first<<std::endl;
      // std::cout<<"------------------------"<<std::endl;
      pts_clr_out.push_back(pts_rot);
    }
  }
  //
  // std::cout<<nx(0)<<" "<<ny(0)<<" "<<nz(0)<<std::endl;
  // std::cout<<R*pts_clr_in[0].first + t<<std::endl;
}
