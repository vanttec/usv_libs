#include "wind_and_waves.h"

WindAndWaves::WindAndWaves(const WindAndWavesParams &params) {
  this->p = params;
  dist_wF = std::normal_distribution<double>(p.mean_wF, p.stddev_wF);
  dist_wN = std::normal_distribution<double>(p.mean_wN, p.stddev_wN);
  dist_dF = std::normal_distribution<double>(p.mean_dF, p.stddev_dF);
  dist_dN = std::normal_distribution<double>(p.mean_dN, p.stddev_dN);
}

ModelState WindAndWaves::update(const vanttec::ModelState &state) {
  double wF = dist_wF(gen);
  double wN = dist_wN(gen);
  double dF_dot = dist_dF(gen);
  double dN_dot = dist_dN(gen);

  double U = std::hypot(state.u, state.v);
  double we = std::abs(w0 - (w0 * w0 / g) * U * std::cos(p.beta_wave));
  double xF2_dot = -we * we * xF1 - 2 * lambda * we * xF2 + Kw * wF;
  xF2 = p.integral_step * (xF2_dot + xF2_dot_last) / 2 + xF2;
  xF2_dot_last = xF2_dot;
  double xF1_dot = xF2;
  xF1 = p.integral_step * (xF1_dot + xF1_dot_last) / 2 + xF1;
  xF1_dot_last = xF1_dot;

  dF = p.integral_step * (dF_dot + dF_dot_last) / 2 + dF;
  dF_dot_last = dF_dot;

  double F_wave = (xF2 + dF) * p.scale_factor;

  double xN2_dot = -we * we * xN1 - 2 * lambda * we * xN2 + Kw * wN;
  xN2 = p.integral_step * (xN2_dot + xN2_dot_last) / 2 + xN2;
  xN2_dot_last = xN2_dot;
  double xN1_dot = xN2;
  xN1 = p.integral_step * (xN1_dot + xN1_dot_last) / 2 + xN1;
  xN1_dot_last = xN1_dot;

  dN = p.integral_step * (dN_dot + dN_dot_last) / 2 + dN;
  dN_dot_last = dN_dot;

  double N_wave = (xN2 + dN) * p.scale_factor;

  double X_wave = F_wave * std::cos(p.beta_wave - state.pose_psi);
  double Y_wave = F_wave * std::sin(p.beta_wave - state.pose_psi);

  double u_w = p.V_wind * std::cos(p.beta_wind - state.pose_psi);
  double v_w = p.V_wind * std::sin(p.beta_wind - state.pose_psi);

  double u_rw = state.u - u_w;
  double v_rw = state.v - v_w;

  double gamma_rw = std::atan2(v_rw, u_rw);
  double V_rw = std::hypot(u_rw, v_rw);

  double CDlaf;
  if (std::abs(gamma_rw) > 1.5708) {
    CDlaf = CDlaf_pi;
  } else {
    CDlaf = CDlaf_0;
  }
  double CDl = CDlaf * AFW / ALW;

  double CX = CDlaf * std::cos(gamma_rw) /
              (1 - ((delta_wind / 2) * (1 - (CDl / CDt)) * (std::sin(2 * gamma_rw) * std::sin(2 * gamma_rw))));
  double CY = CDt * std::sin(gamma_rw) /
              (1 - ((delta_wind / 2) * (1 - (CDl / CDt)) * (std::sin(2 * gamma_rw) * std::sin(2 * gamma_rw))));
  double CN = -0.18 * (gamma_rw - M_PI_2) * CY;

  double X_wind = 0.5 * rho * V_rw * V_rw * CX * AFW;
  double Y_wind = 0.5 * rho * V_rw * V_rw * CY * ALW;
  double N_wind = 0.5 * rho * V_rw * V_rw * CN * AFW * LOA;

  double delta_x = X_wave + X_wind;
  double delta_y = Y_wave + Y_wind;
  double delta_theta = N_wave + N_wind;;

  ModelState outState = state;
  outState.u = 0;
  outState.v = 0;
  outState.r = 0;
  outState.pose_x += delta_x;
  outState.pose_y += delta_y;
  outState.pose_psi += delta_theta;

  return outState;
}