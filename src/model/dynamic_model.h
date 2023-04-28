//
// Created by Abiel on 3/6/23.
//

#ifndef USV_ROS2_DYNAMIC_MODEL_H
#define USV_ROS2_DYNAMIC_MODEL_H

#include <Eigen/Dense>
#include "control/datatypes.h"

using namespace vanttec;

class DynamicModel {
public:
  DynamicModel();
  DynamicModel(double pose_x, double pose_y, double pose_psi);
  ModelState update(double leftThruster, double rightThruster);
  [[nodiscard]] const ModelState& currentState() const {
    return state;
  }

  static double constrainAngle(double angle);

private:
  //Model parameters
  constexpr static const double X_u_dot = -2.25;
  constexpr static const double Y_v_dot = -23.13;
  constexpr static const double Y_r_dot = -1.31;
  constexpr static const double N_v_dot = -16.41;
  constexpr static const double N_r_dot = -2.79;
  constexpr static const double Yvv = -99.99;
  constexpr static const double Yvr = -5.49;
  constexpr static const double Yrv = -5.49;
  constexpr static const double Yrr = -8.8;
  constexpr static const double Nvv = -5.49;
  constexpr static const double Nvr = -8.8;
  constexpr static const double Nrv = -8.8;
  constexpr static const double Nrr = -3.49;
  constexpr static const double m = 30;
  constexpr static const double Iz = 4.1;
  constexpr static const double B = 0.41;
  constexpr static const double c = 0.78;
  constexpr static const double integral_step = 0.01;

  Eigen::Vector3f upsilon = Eigen::Vector3f::Zero();
  Eigen::Vector3f upsilon_dot = Eigen::Vector3f::Zero();
  Eigen::Vector3f upsilon_dot_last = Eigen::Vector3f::Zero();
  Eigen::Vector3f eta = Eigen::Vector3f::Zero();
  Eigen::Vector3f eta_dot_last = Eigen::Vector3f::Zero();
  Eigen::Matrix3f M;

  ModelState state{};
};


#endif //USV_ROS2_DYNAMIC_MODEL_H
