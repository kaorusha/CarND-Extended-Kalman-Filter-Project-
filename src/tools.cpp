#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  // Simplify and reused terms
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2); 
  
  // Calculate a Jacobian here
  if (c1 != 0){
     Hj << (px/c2), (py/c2), 0, 0,
           -(py/c1), (px/c1), 0, 0,
           py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2; 
  } else
     std::cout << "Tools::CalculateJacobian() - Error - Division by Zero" << std::endl;
  
  return Hj;
}
