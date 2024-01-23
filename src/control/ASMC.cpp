//
// Created by Abiel on 3/6/23.
//

#include "ASMC.h"
#include <cmath>
#include <algorithm>

ASMC::ASMC(const ASMCParams &params) {
  this->p = params;
}

ASMCOutput ASMC::update(const ASMCState &s, const ASMCSetpoint &setpoint) {
  double u = s.vel_x;
  double v = s.vel_y;
  double r = s.vel_r;

  double psi = s.theta;

  double beta = std::asin(v / (0.001 + std::hypot(u, v)));
  beta = 0;

  double psi_d = beta + setpoint.heading_setpoint;

  double r_d = (psi_d - psi_d_last) / integral_step;
  psi_d_last = psi_d;
  o_dot_dot = (((r_d - o_last) * f1) - (f3 * o_dot_last)) * f2;
  o_dot = integral_step * (o_dot_dot + o_dot_dot_last) / 2.0 + o_dot;
  o = integral_step * (o_dot + o_dot_last) / 2.0 + o;
  r_d = o;
  o_last = o;
  o_dot_last = o_dot;
  o_dot_dot_last = o_dot_dot;

  double Xu = -25;
  double Xuu = 0;
  if(std::abs(u) > 1.2){
    Xu = 64.55;
    Xuu = -70.92;
  }

  double Nr = 0.02 * (-M_PI * 1000) * std::hypot(u, v) * 0.09 * 0.09 * 1.01 * 1.01;

  // Rewrite USV model in simplified components f and g
  double g_u = 1 / (m - X_u_dot);
  double g_psi = 1 / (Iz - N_r_dot);
  double f_u = ((m - Y_v_dot) * v * r + (Xuu * std::abs(u) + Xu * u)) / (m - X_u_dot);
  double f_psi = (((-X_u_dot + Y_v_dot) * u * v + (Nr * r)) / (Iz - N_r_dot));

  double e_psi = psi_d - psi;
  double e_psi_dot = r_d - r;

  double u_d = setpoint.velocity_setpoint;
  double e_u = u_d - u;
  e_u_int = integral_step * (e_u + e_u_last) / 2.0 + e_u_int;
  e_u_last = e_u;

  // Sliding surfaces for speed and heading
  double sigma_u = e_u + p.lambda_u * e_u_int;
  double sigma_psi = e_psi_dot + p.lambda_psi * e_psi;

  // Compute gain derivs
  double ka_dot_u, ka_dot_psi;
  if(ka_u > p.kmin_u){
    ka_dot_u = std::copysign(p.k_u, std::abs(sigma_u) - p.mu_u);
  } else {
    ka_dot_u = p.kmin_u;
  }

  if(ka_psi > p.kmin_psi){
    ka_dot_psi = std::copysign(p.k_psi, std::abs(sigma_psi) - p.mu_psi);
  } else {
    ka_dot_psi = p.kmin_psi;
  }

  // Compute gains
  ka_u = integral_step * (ka_dot_u + ka_dot_u_last) / 2.0 + ka_u;
  ka_dot_u_last = ka_dot_u;

  ka_psi = integral_step * (ka_dot_psi + ka_dot_psi_last) / 2.0 + ka_psi;
  ka_dot_psi_last = ka_dot_psi;

  //Compute ASMC for speed and heading
  double ua_u = (-ka_u * std::copysign(std::pow(std::abs(sigma_u), 0.5), sigma_u)) - (p.k2_u * sigma_u);
  double ua_psi = (-ka_psi * std::copysign(std::pow(std::abs(sigma_psi), 0.5), sigma_psi)) - (p.k2_psi * sigma_psi);

  //Compute control inputs for speed and heading
  double Tx = ((p.lambda_u * e_u) - f_u - ua_u) / g_u;
  double Tz = ((p.lambda_psi * e_psi) - f_psi - ua_psi) / g_psi;

  double Tport = (Tx / 2.0) + (Tz / B);
  double Tstbd = (Tx / (2 * c)) - (Tz / (B * c));

  if(setpoint.pivot_enabled == 1){
    if(Tport > Tstbd){
      Tstbd = - 5*Tport;
      Tport/=10; // 2
    } else {
      Tport = - 5*Tstbd;
      Tstbd/=10; // 2
    }
  }

  Tport = std::clamp(Tport, -30.0, 36.0);
  Tstbd = std::clamp(Tstbd, -30.0, 36.0);

  ASMCOutput out{};
  out.left_thruster = Tport;
  out.right_thruster = Tstbd;

  out.speed_setpoint = u_d;
  out.heading_setpoint = psi_d;

  out.speed_gain = ka_u;
  out.heading_gain = ka_psi;

  out.speed_error = e_u;
  out.heading_error = e_psi;

  out.speed_sigma = sigma_u;
  out.heading_sigma = sigma_psi;

  out.Tx = Tx;
  out.Tz = Tz;

  return out;
}

double ASMC::constrainAngle(double angle) {
  angle = std::copysign(std::fmod(angle, 2 * M_PI), angle);
  if (angle > M_PI) {
    angle -= 2 * M_PI;
  } else if (angle < -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}

ASMCParams ASMC::defaultParams() {
  ASMCParams p{};
  p.k_u = 0.1;
  p.k_psi = 0.175;
  p.kmin_u = 0.05;
  p.kmin_psi = 0.2;
  p.k2_u = 0.02;
  p.k2_psi = 0.1;
  p.mu_u = 0.05;
  p.mu_psi = 0.1;
  p.lambda_u = 0.001;
  p.lambda_psi = 1.0;
  return p;
}
