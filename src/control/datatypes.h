#pragma once
namespace vanttec{
  struct ControllerOutput {
    // Main output data, left and right thruster
    double left_thruster, right_thruster;
    double Tx, Tz;
  };

  struct ControllerState {
    double u, v, r;
    double psi;
  };
  
  struct USVModel {
    double X_u_dot;
    double Y_v_dot;
    double N_r_dot;
    double Yvv;
    double Nrr;
    double m; //mass
    double Iz; //moment of inertia
    double B; //centerline-to-centerline separation
    double c; //thruster correction factor

    static const USVModel getBarcolomeo(){
      USVModel model;
      model.X_u_dot = -2.25;
      model.Y_v_dot = -23.13;
      model.N_r_dot = -2.79;
      model.Yvv = -99.99;
      model.Nrr = -3.49;
      model.m = 30.0;
      model.Iz = 4.1;
      model.B = 0.41;
      model.c = 0.78;
      return model;
    }

  };
}