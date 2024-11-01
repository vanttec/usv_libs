#pragma once
#include "datatypes.h"

using namespace vanttec;

struct AITSMCNEWParams {
  double k_u, k_psi,
        epsilon_u,
        epsilon_psi,
        k_alpha_u,
        k_alpha_psi,
        k_beta_u,
        k_beta_psi,
        tc_u,
        tc_psi,
        q_u,
        q_psi,
        p_u,
        p_psi,
        adaptive;
};

struct AITSMCNEWSetpoint{
  double u{0}, psi{0}, psi_dot{0};
  double dot_u{0}, dot_r{0};
};

struct AITSMCNEWDebugData {
  double e_u{0}, e_psi{0};
  double edot_psi{0}, ei_psi{0};
  double s_u{0}, s_psi{0};
  double Ka_u{0}, Ka_psi{0};
  double Tx{0}, Tz{0};
};

class AITSMC_NEW {
public:
  explicit AITSMC_NEW(const AITSMCNEWParams &params);

  ControllerOutput update(const ControllerState &s, const AITSMCNEWSetpoint &setpoint);

  static AITSMCNEWParams defaultParams();

  double normalize_angle(double angle_in);
  double angle_dist(double ang1, double ang2);

  [[nodiscard]] AITSMCNEWDebugData getDebugData() const {
    return debugData;
  }

private:
  AITSMCNEWParams p;

  double m = 30; // Mass
  double Iz = 4.1;
  double B = 0.41;
  double c = 0.78;

  static constexpr double integral_step{0.01};
  double eidot_u_last{0}, eidot_psi_last{0};
  double e_psi_last{0}, e_psi_last_last{0};

  double alpha_u{0}, alpha_psi{0};
  double beta_psi{0}, gamma_psi{0};
  double Ka_u{0}, Ka_psi{0};
  double ei_u{0}, ei_psi{0};

  double Ka_dot_last_u{0}, Ka_dot_last_psi{0};

  AITSMCNEWDebugData debugData;

  USVModel model = USVModel::getBarcolomeo(); // TODO make this configurable
  double g_u{0}, g_psi{0};
};