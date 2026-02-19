#include "AITSMC_NEW.h"
#include <cmath>
#include <algorithm>
#include <iostream>

AITSMC_NEW::AITSMC_NEW(const AITSMCNEWParams &params) {
  this->p = params;
  g_u = (1 / (model.m - model.X_u_dot));
  g_psi = (1 / (model.Iz - model.N_r_dot));
}

AITSMCNEWParams AITSMC_NEW::defaultParams() {
  AITSMCNEWParams p;
  p.k_u = 0.1;
  p.k_psi = 0.1;
  p.epsilon_u = 0.1;
  p.epsilon_psi = 0.1;
  p.k_alpha_u = 0.5;
  p.k_alpha_psi = 0.5;
  p.k_beta_u = 0.5;
  p.k_beta_psi = 0.5;
  p.tc_u = 2.0;
  p.tc_psi = 2;
  p.q_u = 3.0;
  p.q_psi = 3.0;
  p.p_u = 5.0;
  p.p_psi = 5.0;
  p.adaptive = 1.0;
  return p;
}

double AITSMC_NEW::normalize_angle(double angle_in){
  double angle_out = std::fmod(angle_in + M_PI, 2 * M_PI);
  if(angle_out < 0){
    angle_out += 2*M_PI;
  }
  return angle_out - M_PI;
}

double AITSMC_NEW::angle_dist(double ang1, double ang2){
  double diff = ang1 - ang2;
  return normalize_angle(diff);
}

ControllerOutput AITSMC_NEW::update(const vanttec::ControllerState &s, const AITSMCNEWSetpoint &setpoint) {
  double Xu = -25;
  double Xuu = 0;
  if (std::abs(s.u) > 1.2) {
    Xu = 64.55;
    Xuu = -70.92;
  }

  double sideslip_angle = std::asin(s.v / (0.001 + std::hypot(s.u, s.v)));
  sideslip_angle = 0;

  double Nr = (-0.52) * std::hypot(s.u, s.v);
  double f_u = (((m - model.Y_v_dot) * s.v * s.r + (Xuu * std::abs(s.u) * s.u + Xu * s.u)) / (m - model.X_u_dot));
  double f_psi = (((-model.X_u_dot + model.Y_v_dot) * s.u * s.v + (model.Nrr * std::abs(s.r) * s.r + Nr * s.r)) /
                (Iz - model.N_r_dot));

  double e_u = setpoint.u - s.u;

  // Second order error (yaw rate)
  // double e_psi = angle_dist(setpoint.psi, s.psi);
  // Same but including 
  double e_psi = angle_dist(setpoint.psi + sideslip_angle, s.psi);
  std::cout << "SIDESLIP: " << sideslip_angle << std::endl;
  // TODO: PRINT THE SIDESLIP VARIABLE, DO FURTHER DEBUGGING TO KNOW WHY EVERYTHING F'S UP WHEN THE BOAT GOES FAST 

  // First order error (heading)
  // double e_psi = setpoint.psi - s.r;

  double sign_u = copysign(e_u != 0 ? 1 : 0, e_u);
  double sign_psi = copysign(e_psi != 0 ? 1 : 0, e_psi);

  double eidot_u = sign_u * std::pow(std::abs(e_u), p.q_u / p.p_u);
  double eidot_psi = sign_psi * std::pow(std::abs(e_psi), p.q_psi / p.p_psi);
  ei_u = integral_step * (eidot_u + eidot_u_last) / 2.0 + ei_u;
  eidot_u_last = eidot_u;
  ei_psi = integral_step * (eidot_psi + eidot_psi_last) / 2.0 + ei_psi;
  // ei_psi = std::clamp(ei_psi, -M_PI, M_PI);
  eidot_psi_last = eidot_psi;

  double edot_psi = setpoint.psi_dot - s.r;
  e_psi_last_last = e_psi_last;
  e_psi_last = e_psi;

  // TODO find how to initialize vars
  static int starting = 1;
  if(starting == 0){
    double e_u0 = e_u;
    double e_psi0 = e_psi;
    alpha_u = (std::pow(std::abs(e_u0),1-p.q_u/p.p_u))/(p.tc_u*(1-p.q_u/p.p_u));
    alpha_psi = (std::pow(std::abs(e_psi0),1-p.q_psi/p.p_psi))/(p.tc_psi*(1-p.q_psi/p.p_psi));
    alpha_u = 0.0001;
    // alpha_psi = 0.0001;
    if(p.adaptive == 1){
      alpha_psi = 0.01;
      beta_psi = 10.;
    } else {
      alpha_psi = 0.01;
      beta_psi = 10.;
    }
    // beta_psi = 0.;
    ei_u = -e_u0 / alpha_u;
    ei_psi = -e_psi0 / alpha_psi;
    if(alpha_u < 0.0001) ei_u = 0;
    if(alpha_psi < 0.0001) ei_psi = 0;
    starting = 2;
    std::cout << "huh" << ei_u << std::endl;
    std::cout << "huhh" << alpha_psi << std::endl;
  }

  double s_u = e_u + alpha_u * ei_u;

  // Second order sliding surface
  double s_psi = edot_psi + beta_psi * e_psi + alpha_psi * ei_psi;

  // First order sliding surface
  // double s_psi = e_psi + alpha_psi * ei_psi;

  // std::cout << "s_psi -> " << s_psi << "\ne_psi: " << e_psi 
  //   << "\nei_psi: " << ei_psi << "\nedot_psi: " << edot_psi 
  //   << "\nalpha_psi: " << alpha_psi
  //   << std::endl;
  // std::cout << "alpha_psi -> " << alpha_psi
  //   << "\nalpha_u -> " << alpha_u
  //   << std::endl;

  double Ka_dot_u{0}, Ka_dot_psi{0};
  // This is just so i dont havee to add another parameter, TODO go back to the first one
  // if(p.adaptive != 0){
  if(1. != 0){
    Ka_dot_u = std::sqrt(p.k_alpha_u) * std::sqrt(std::fabs(s_u)) 
      - std::sqrt(p.k_beta_u) * std::pow(Ka_u, 2);
    Ka_dot_psi = std::sqrt(p.k_alpha_psi) * std::sqrt(std::fabs(s_psi)) 
      - std::sqrt(p.k_beta_psi) * std::pow(Ka_psi, 2);
    // Update Integrals
    Ka_u = integral_step * (Ka_dot_u + Ka_dot_last_u) / 2 + Ka_u;
    Ka_dot_last_u = Ka_dot_u;

    Ka_psi = integral_step * (Ka_dot_psi + Ka_dot_last_psi) / 2 + Ka_psi;
    Ka_dot_last_psi = Ka_dot_psi;
  }


  double sign_s_u = std::copysign(s_u == 0 ? 0 : 1, s_u);
  double ua_u = (-Ka_u * std::sqrt(std::abs(s_u)) * sign_s_u) - (Ka_u * p.epsilon_u *  s_u);
  // double ua_u = (-Ka_u * std::sqrt(std::abs(s_u)) * sign_s_u) - (Ka_u * p.epsilon_u *  std::pow(std::abs(s_u), 0.5) * sign_s_u);

  double sign_s_psi = std::copysign(s_psi == 0 ? 0 : 1, s_psi);
  double ua_psi = (-Ka_psi * std::sqrt(std::abs(s_psi)) * sign_s_psi) - (Ka_psi * p.epsilon_psi * s_psi);

  double Tx = (setpoint.dot_u + (alpha_u * eidot_u) - f_u - ua_u) / g_u;

  // Second order control output
  double Tz = (setpoint.dot_r + (beta_psi * edot_psi) + (alpha_psi * eidot_psi) - f_psi - ua_psi) / g_psi;

  // First order control output
  // double Tz = (setpoint.dot_r + (alpha_psi * eidot_psi) - f_psi - ua_psi) / g_psi;

  // Clamp forces
  // Tx = std::clamp(Tx, -60.0, 73.0);
  // Tz = std::clamp(Tz, -14.0, 14.0);

  if(starting == 1){
    Tx = 0;
    Tz = 0;
    Ka_u = 0;
    Ka_dot_last_u = 0;

    Ka_psi = 0;
    Ka_dot_last_psi = 0;

    ei_u = -e_u / alpha_u;
    eidot_u_last = 0;

    ei_psi = -e_psi / alpha_psi;
    eidot_psi_last = 0;
    // edot_psi = 0;

    starting = 0;
  }

  double port_t = (Tx / 2.0) + (Tz / B);
  double starboard_t = (Tx / (2.0 * c)) - (Tz / (B * c));

  port_t = std::clamp(port_t, -60.0, 73.0);
  starboard_t = std::clamp(starboard_t, -60.0, 73.0);

  ControllerOutput out{};
  out.left_thruster= port_t;
  out.right_thruster = starboard_t;

  out.Tx = Tx;
  out.Tz = Tz;

  debugData.e_u = e_u;
  debugData.e_psi = e_psi;
  // debugData.edot_psi = edot_psi;
  debugData.ei_psi = eidot_psi;
  debugData.s_u = s_u;
  debugData.s_psi = s_psi;
  debugData.Ka_u = Ka_u;
  debugData.Ka_psi = Ka_psi;
  debugData.Tx = Tx;
  debugData.Tz = Tz;

  return out;
}