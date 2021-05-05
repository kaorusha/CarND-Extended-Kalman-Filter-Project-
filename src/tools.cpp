#include "tools.h"

#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;
  if (estimations.size() == 0) {
    std::cout << "Invalid estimation data: the estimation vector size should "
                 "not be zero"
              << std::endl;
    return rmse;
  }
  for (int i = 0; i < estimations.size(); ++i) {
    // check size
    if (estimations[i].size() != ground_truth[i].size()) {
      std::cout << "Invalid data: the estimation vector size should equal "
                   "ground truth vector size"
                << std::endl;
      return rmse;
    }

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }
  // calculate the mean
  rmse = rmse / estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3, 4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Simplify and reused terms
  float c1 = px * px + py * py;
  float c2 = sqrt(c1);
  float c3 = (c1 * c2);

  // Calculate a Jacobian here
  if (c1 != 0) {
    Hj << (px / c2), (py / c2), 0, 0, -(py / c1), (px / c1), 0, 0,
        py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2,
        py / c2;
  } else
    std::cout << "Tools::CalculateJacobian() - Error - Division by Zero"
              << std::endl;

  return Hj;
}
