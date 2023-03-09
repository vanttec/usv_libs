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
  u_d = setpoint.velocity_setpoint;
  psi_d = setpoint.heading_setpoint;

  double Xu = -25;
  double Xuu = 0;

  const double u = s.vel_x;
  const double u_abs = std::abs(u);
  const double v = s.vel_y;
  const double r = s.vel_r;

  if(u_abs > 1.2) {
    Xu = 64.55;
    Xuu = -70.92;
  }

  const double Nr = -0.52 * std::hypot(u, v);
  const double g_u = (1.0 / (m - X_u_dot));
  const double g_psi = (1.0 / (Iz - N_r_dot));

  double f_u =
          (((m - Y_v_dot) * v * r + (Xuu * u_abs * u + Xu * u)) / (m - X_u_dot));
  double f_psi = (((-X_u_dot + Y_v_dot) * u * v + (Nr * r)) / (Iz - N_r_dot));

  double e_u = u_d - u;
  double e_psi = constrainAngle(psi_d - s.theta);

  e_u_int = (integral_step) * (e_u + e_u_last) / 2 +
            e_u_int;  // integral of the surge speed error
  e_u_last = e_u;

  double psi_d_dif = constrainAngle(psi_d - psi_d_last);

  double r_d = (psi_d_dif) / integral_step;
  psi_d_last = psi_d;
  o_dot_dot = (((r_d - o_last) * f1) - (f3 * o_dot_last)) * f2;
  o_dot = (integral_step) * (o_dot_dot + o_dot_dot_last) / 2 + o_dot;
  o = (integral_step) * (o_dot + o_dot_last) / 2 + o;
  r_d = o;
  o_last = o;
  o_dot_last = o_dot;
  o_dot_dot_last = o_dot_dot;

  double e_psi_dot = r_d - r;

  double sigma_u = e_u + p.lambda_u * e_u_int;
  double sigma_psi = e_psi_dot + p.lambda_psi * e_psi;
  // float sigma_psi = 0.1 * e_psi_dot + lambda_psi * e_psi;

  double sigma_u_abs = std::abs(sigma_u);
  double sigma_psi_abs = std::abs(sigma_psi);

  double sign_u_sm = 0;
  double sign_psi_sm = 0;
  double Ka_dot_u = 0;

  if (Ka_u > p.kmin_u) {
    double signvar = sigma_u_abs - p.mu_u;
    if (signvar == 0) {
      sign_u_sm = 0;
    } else {
      sign_u_sm = std::copysign(1, signvar);
    }
    Ka_dot_u = p.k_u * sign_u_sm;
  } else {
    Ka_dot_u = p.kmin_u;
  }

  if (Ka_psi > p.kmin_psi) {
    double signvar = sigma_psi_abs - p.mu_psi;
    if (signvar == 0) {
      sign_psi_sm = 0;
    } else {
      sign_psi_sm = copysign(1.0, signvar);
    }
    Ka_dot_psi = p.k_psi * sign_psi_sm;
  } else {
    Ka_dot_psi = p.kmin_psi;
  }

  Ka_u = (integral_step) * (Ka_dot_u + Ka_dot_last_u) / 2 +
         Ka_u;  // integral to get the speed adaptative gain
  Ka_dot_last_u = Ka_dot_u;

  Ka_psi = (integral_step) * (Ka_dot_psi + Ka_dot_last_psi) / 2.0 +
           Ka_psi;  // integral to get the heading adaptative gain
  Ka_dot_last_psi = Ka_dot_psi;

  int sign_u = 0;
  int sign_psi = 0;

  if (sigma_u == 0) {
    sign_u = 0;
  } else {
    sign_u = std::copysign(1, sigma_u);
  }
  double ua_u = ((-Ka_u) * std::sqrt(sigma_u_abs) * sign_u) - (p.k2_u * sigma_u);

  if (sigma_psi == 0) {
    sign_psi = 0;
  } else {
    sign_psi = std::copysign(1, sigma_psi);
  }
  double ua_psi = ((-Ka_psi) * std::sqrt(sigma_psi_abs) * sign_psi) -
                  (p.k2_psi * sigma_psi);

  double Tx = ((p.lambda_u * e_u) - f_u - ua_u) / g_u;  // surge force
  double Tz =
          ((p.lambda_psi * e_psi_dot) - f_psi - ua_psi) / g_psi;  // yaw rate moment

  Tx = std::clamp(Tx, -60.0, 73.0);
  Tz = std::clamp(Tz, -14.0, 14.0);

  if (u_d == 0) {
    Tx = 0;
    Tz = 0;
    Ka_u = 0;
    Ka_dot_last_u = 0;
    Ka_psi = 0;
    Ka_dot_last_psi = 0;
    e_u_int = 0;
    e_u_last = 0;
    o_dot_dot = 0;
    o_dot = 0;
    o = 0;
    o_last = 0;
    o_dot_last = 0;
    o_dot_dot_last = 0;
    psi_d = s.theta;
    psi_d_last = s.theta;
  }

  double port_t = (Tx / 2.0) + (Tz / B);
  double starboard_t = (Tx / (2.0 * c)) - (Tz / (B * c));

  starboard_t = std::clamp(starboard_t, -30.0, 36.5);
  port_t = std::clamp(port_t, -30.0, 36.5);

  ASMCOutput out{};

  out.left_thruster = port_t;
  out.right_thruster = starboard_t;

  out.speed_gain = Ka_u;
  out.speed_error = e_u;
  out.speed_sigma = sigma_u;

  out.heading_gain = Ka_psi;
  out.heading_error = e_psi;
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
  p.k_psi = 0.2;
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