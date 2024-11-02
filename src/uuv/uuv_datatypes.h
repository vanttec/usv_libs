#include <eigen3/Eigen/Dense>
#include <array>

#pragma once
namespace uuv{
  // struct ControllerOutput {
  //   // Main output data, left and right thruster
  //   double left_thruster, right_thruster;
  //   double Tx, Tz;
  // };

  // struct ControllerState {
  //   double u, v, r;
  //   double psi;
  // };
  
  struct UUVState{
    Eigen::VectorXf eta{0., 0., 0., 0., 0., 0.,};
    Eigen::VectorXf eta_dot{0., 0., 0., 0., 0., 0.,};
    Eigen::VectorXf eta_dot_prev{0., 0., 0., 0., 0., 0.,};
    Eigen::VectorXf nu{0., 0., 0., 0., 0., 0.,};
    Eigen::VectorXf nu_dot{0., 0., 0., 0., 0., 0.,};
    Eigen::VectorXf nu_dot_prev{0., 0., 0., 0., 0., 0.,};
  };
}