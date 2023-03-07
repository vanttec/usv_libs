//
// Created by Abiel on 3/6/23.
//

#include "dynamic_model.h"

DynamicModel::DynamicModel(double pose_x, double pose_y, double pose_psi) {
  eta(0) = pose_x; eta(1) = pose_y; eta(2) = pose_psi;
  M << m - X_u_dot, 0, 0, 0, m - Y_v_dot, 0 - Y_r_dot, 0, 0 - N_v_dot,
          Iz - N_r_dot;
}

DynamicModel::DynamicModel() : DynamicModel(0, 0, 0) {
  ;
}

DynamicModelOutput DynamicModel::update(double leftThruster, double rightThruster) {
  double Xu = -25;
  double Xuu = 0;
  auto upsilon_abs = upsilon.cwiseAbs();
  if (upsilon(0) > 1.2) {
    Xu = 64.55;
    Xuu = -70.92;
  }

  double vel = std::hypot(upsilon(0), upsilon(1));

  double Yv = 0.5 * (-40.0 * 1000.0 * std::abs(upsilon(1))) *
              (1.1 + 0.0045 * (1.01 / 0.09) - 0.1 * (0.27 / 0.09) +
               0.016 * (std::pow((0.27 / 0.09), 2.0)));
  double Yr = 6 * (-3.141592 * 1000) * vel * 0.09 * 0.09 * 1.01;
  double Nv = 0.06 * (-3.141592 * 1000) * vel * 0.09 * 0.09 * 1.01;
  double Nr = 0.02 * (-3.141592 * 1000) * vel * 0.09 * 0.09 * 1.01 * 1.01;

  Eigen::Vector3f T;
  T << leftThruster + c * rightThruster, 0, 0.5 * B * (leftThruster - c * rightThruster);

  Eigen::Matrix3f CRB, CA, C;
  CRB << 0, 0, 0 - m * upsilon(1), 0, 0, m * upsilon(0), m * upsilon(1),
          0 - m * upsilon(0), 0;

  CA << 0, 0,
          2 * ((Y_v_dot * upsilon(1)) + ((Y_r_dot + N_v_dot) / 2) * upsilon(2)),
          0, 0, 0 - X_u_dot * m * upsilon(0),
          2 * (((0 - Y_v_dot) * upsilon(1)) -
               ((Y_r_dot + N_v_dot) / 2) * upsilon(2)),
          X_u_dot * m * upsilon(0), 0;

  C = CRB + CA;

  Eigen::Matrix3f Dl, Dn, D;
  Dl << 0 - Xu, 0, 0, 0, 0 - Yv, 0 - Yr, 0, 0 - Nv, 0 - Nr;

  Dn << Xuu * upsilon_abs(0), 0, 0, 0,
          Yvv * upsilon_abs(1) + Yvr * upsilon_abs(2),
          Yrv * upsilon_abs(1) + Yrr * upsilon_abs(2), 0,
          Nvv * upsilon_abs(1) + Nvr * upsilon_abs(2),
          Nrv * upsilon_abs(1) + Nrr * upsilon_abs(2);

  D = Dl - Dn;

  upsilon_dot = M.inverse() * (T - C * upsilon - D * upsilon);
  upsilon = integral_step * (upsilon_dot + upsilon_dot_last) / 2 +
            upsilon;  // integral
  upsilon_dot_last = upsilon_dot;

  Eigen::Matrix3f J;
  Eigen::Vector3f eta_dot;
  J << std::cos(eta(2)), -std::sin(eta(2)), 0, std::sin(eta(2)),
          std::cos(eta(2)), 0, 0, 0, 1;

  eta_dot = J * upsilon;  // transformation into local reference frame
  eta = integral_step * (eta_dot + eta_dot_last) / 2 + eta;  // integral
  eta_dot_last = eta_dot;

  DynamicModelOutput out;
  out.pose_x = eta(0);
  out.pose_y = eta(1);
  out.pose_psi = eta(2);

  out.vel_x = upsilon(0);
  out.vel_y = upsilon(1);
  out.vel_r = upsilon(2);

  state = out;
  return out;
}