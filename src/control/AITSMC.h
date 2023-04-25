#pragma once
#include "datatypes.h"

using namespace vanttec;

struct AITSMCParams {
  double k_u,
        k_r,
        kmin_u,
        kmin_r,
        k2_u,
        k2_r,
        mu_u,
        mu_r,
        tc_u,
        tc_r,
        q_u,
        q_r,
        p_u,
        p_r;
};

struct AITSMCSetpoint{
  double u{0}, r{0};
  double dot_u{0}, dot_r{0};
};

struct AITSMCDebugData {
  double e_u{0}, e_r{0};
  double s_u{0}, s_r{0};
  double Ka_u{0}, Ka_r{0};
  double Tx{0}, Tz{0};
};

class AITSMC {
public:
  explicit AITSMC(const AITSMCParams &params);

  ControllerOutput update(const ControllerState &s, const AITSMCSetpoint &setpoint);

  static AITSMCParams defaultParams();

  [[nodiscard]] AITSMCDebugData getDebugData() const {
    return debugData;
  }

private:
  AITSMCParams p;

  double m = 30; // Mass
  double Iz = 4.1;
  double B = 0.41;
  double c = 0.78;

  static constexpr double integral_step{0.01};
  double eidot_u_last{0}, eidot_r_last{0};

  double alpha_u{0}, alpha_r{0};
  double Ka_u{0}, Ka_r{0};
  double ei_u{0}, ei_r{0};

  double Ka_dot_last_u{0}, Ka_dot_last_r{0};

  AITSMCDebugData debugData;

  USVModel model = USVModel::getBarcolomeo(); // TODO make this configurable
  double g_u{0}, g_r{0};
};