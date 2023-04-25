#include "AITSMC.h"
#include <cmath>
#include <algorithm>

AITSMC::AITSMC(const AITSMCParams &params) {
  this->p = params;
  g_u = (1 / (model.m - model.X_u_dot));
  g_r = (1 / (model.Iz - model.N_r_dot));
}

AITSMCParams AITSMC::defaultParams() {
  AITSMCParams p;
  p.k_u = 1.0;
  p.k_r = 1.0;
  p.kmin_u = 0.01;
  p.kmin_r = 0.01;
  p.k2_u = 0.01;
  p.k2_r = 0.01;
  p.mu_u = 0.001;
  p.mu_r = 0.001;
  p.tc_u = 2.0;
  p.tc_r = 2;
  p.q_u = 3.0;
  p.q_r = 3.0;
  p.p_u = 5.0;
  p.p_r = 5.0;
  return p;
}

ControllerOutput AITSMC::update(const vanttec::ControllerState &s, const AITSMCSetpoint &setpoint) {
  double Xu = -25;
  double Xuu = 0;
  if (std::abs(s.u) > 1.2) {
    Xu = 64.55;
    Xuu = -70.92;
  }

  double Nr = (-0.52) * std::hypot(s.u, s.v);
  double f_u = (((m - model.Y_v_dot) * s.v * s.r + (Xuu * std::abs(s.u) * s.u + Xu * s.u)) / (m - model.X_u_dot));
  double f_r = (((-model.X_u_dot + model.Y_v_dot) * s.u * s.v + (model.Nrr * std::abs(s.r) * s.r + Nr * s.r)) /
                (Iz - model.N_r_dot));

  double e_u = setpoint.u - s.u;
  double e_r = setpoint.r - s.r;

  double sign_u = copysign(e_u != 0 ? 1 : 0, e_u);
  double sign_r = copysign(e_r != 0 ? 1 : 0, e_r);

  double eidot_u = sign_u * std::pow(std::abs(e_u), p.q_u / p.p_u);
  double eidot_r = sign_r * std::pow(std::abs(e_r), p.q_r / p.p_r);
  ei_u = integral_step * (eidot_u + eidot_u_last) / 2.0 + ei_u;
  eidot_u_last = eidot_u;
  ei_r = integral_step * (eidot_r + eidot_r_last) / 2.0 + ei_r;
  eidot_r_last = eidot_r;

  // TODO find how to initialize vars
  static int starting = 1;
  if(starting == 0){
    double e_u0 = e_u;
    double e_r0 = e_r;
    alpha_u = (std::pow(std::abs(e_u0),1-p.q_u/p.p_u))/(p.tc_u*(1-p.q_u/p.p_u));
    alpha_r = (std::pow(std::abs(e_r0),1-p.q_r/p.p_r))/(p.tc_r*(1-p.q_r/p.p_r));
    ei_u = -e_u0 / alpha_u;
    ei_r = -e_r0 / alpha_r;
    if(alpha_u < 0.0001) ei_u = 0;
    if(alpha_r < 0.0001) ei_r = 0;
    starting = 2;
  }

  double s_u = e_u + alpha_u * ei_u;
  double s_r = e_r + alpha_r * ei_r;

  double Ka_dot_u{0}, Ka_dot_r{0};
  if(Ka_u > p.kmin_u){
    double var = std::abs(s_u) - p.mu_u;
    double sign_u_sm = std::copysign(var == 0 ? 0 : 1, var);
    Ka_dot_u = p.k_u * sign_u_sm;
  } else {
    Ka_dot_u = p.kmin_u;
  }

  // Update Ka_u
  Ka_u = integral_step * (Ka_dot_u + Ka_dot_last_u) / 2 + Ka_u;
  Ka_dot_last_u = Ka_dot_u;

  if(Ka_r > p.kmin_r){
    double var = std::abs(s_r) - p.mu_r;
    double sign_r_sm = std::copysign(var == 0 ? 0 : 1, var);
    Ka_dot_r = p.k_r * sign_r_sm;
  } else {
    Ka_dot_r = p.kmin_r;
  }

  Ka_r = integral_step * (Ka_dot_r + Ka_dot_last_r) / 2 + Ka_r;
  Ka_dot_last_r = Ka_dot_r;

  double sign_su = std::copysign(s_u == 0 ? 0 : 1, s_u);
  double ua_u = (-Ka_u * std::sqrt(std::abs(s_u)) * sign_su) - (p.k2_u * s_u);

  double sign_sr = std::copysign(s_r == 0 ? 0 : 1, s_r);
  double ua_r = (-Ka_r * std::sqrt(std::abs(s_r)) * sign_sr) - (p.k2_r * s_r);

  double Tx = (setpoint.dot_u + (alpha_u * eidot_u) - f_u - ua_u) / g_u;
  double Tz = (setpoint.dot_r + (alpha_r * eidot_r) - f_r - ua_r) / g_r;

  // Clamp forces
  Tx = std::clamp(Tx, -60.0, 73.0);
  Tz = std::clamp(Tz, -14.0, 14.0);

  if(starting == 1){
    Tx = 0;
    Tz = 0;
    Ka_u = p.kmin_u;
    Ka_dot_last_u = 0;

    Ka_r = p.kmin_r;
    Ka_dot_last_r = 0;

    ei_u = -e_u / alpha_u;
    eidot_u_last = 0;

    ei_r = -e_r / alpha_r;
    eidot_r_last = 0;
    starting = 0;
  }

  double port_t = (Tx / 2.0) + (Tz / B);
  double starboard_t = (Tx / (2.0 * c)) - (Tz / (B * c));

  port_t = std::clamp(port_t, -30.0, 36.5);
  starboard_t = std::clamp(starboard_t, -30.0, 36.5);

  ControllerOutput out{};
  out.left_thruster= port_t;
  out.right_thruster = starboard_t;

  out.Tx = Tx;
  out.Tz = Tz;

  debugData.e_u = e_u;
  debugData.e_r = e_r;
  debugData.s_u = s_u;
  debugData.s_r = s_r;
  debugData.Ka_u = Ka_u;
  debugData.Ka_r = Ka_r;
  debugData.Tx = Tx;
  debugData.Tz = Tz;

  return out;
}