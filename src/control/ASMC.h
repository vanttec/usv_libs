//
// Created by Abiel on 3/6/23.
//

#ifndef USV_ROS2_ASMC_H
#define USV_ROS2_ASMC_H

struct ASMCParams {
  double k_u,
          k_psi,
          kmin_u,
          kmin_psi,
          k2_u,
          k2_psi,
          mu_u,
          mu_psi,
          lambda_u,
          lambda_psi;
};

struct ASMCSetpoint {
  double heading_setpoint, velocity_setpoint;
};

struct ASMCState {
  double vel_x, vel_y, vel_r;
  double theta;
};

struct ASMCOutput {
  // Main output data, left and right thruster
  double left_thruster, right_thruster;

  // Debug data
  double speed_setpoint, heading_setpoint;
  double speed_gain, speed_error, speed_sigma;
  double heading_gain, heading_error, heading_sigma;
  double Tx, Tz;
};

class ASMC {
public:
  explicit ASMC(const ASMCParams &params);

  ASMCOutput update(const ASMCState &s, const ASMCSetpoint &setpoint);

  // Returns default parameters used for USV
  static ASMCParams defaultParams();

  static double constrainAngle(double angle);

private:
  ASMCParams p;
  double e_u_last{0}, e_u_int{0};
  double psi_d_last{0};
  double o_last{0}, o_dot_last{0}, o_dot_dot_last{0}, o_dot{0}, o_dot_dot{0}, o{0};

  double ka_psi{0}, ka_u{0};
  double ka_dot_u_last{0}, ka_dot_psi_last{0};

  constexpr static const double X_u_dot = -2.25;
  constexpr static const double Y_v_dot = -23.13;
  constexpr static const double N_r_dot = -2.79;
  constexpr static const double m = 30;
  constexpr static const double Iz = 4.1;
  constexpr static const double B = 0.41;
  constexpr static const double c = 0.78;;

  //Second order filter gains (for r_d)
  constexpr static const double f1 = 2.0;
  constexpr static const double f2 = 2.0;
  constexpr static const double f3 = 2.0;

  constexpr static const double integral_step = 0.01;
};


#endif //USV_ROS2_ASMC_H
