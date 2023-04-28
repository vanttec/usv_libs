#pragma once
#include <random>
#include "control/datatypes.h"
using namespace vanttec;

struct WindAndWavesParams {
  double beta_wave;
  double mean_wF, mean_dF;
  double mean_wN, mean_dN;
  double stddev_wF, stddev_dF;
  double stddev_wN, stddev_dN;
  double beta_wind;
  double V_wind;
  double scale_factor;
  double integral_step{0.01};

  static WindAndWavesParams defaultParams() {
    WindAndWavesParams p;
    p.beta_wave = 0.0;
    p.stddev_wF = 1.0;
    p.stddev_dF = 0.5;
    p.stddev_wN = 0.2;
    p.stddev_dN = 0.1;
    p.mean_wF = 0;
    p.mean_dF = 0;
    p.mean_wN = 0;
    p.mean_dN = 0;
    p.beta_wind = 0.0;
    p.V_wind = 5.11;
    p.scale_factor = 1.0;
    p.integral_step = 0.01;
    return p;
  }
};

class WindAndWaves{
public:
  WindAndWaves(const WindAndWavesParams &params = WindAndWavesParams::defaultParams());

  /*
   * Takes in current state, returns perturbed pose
   */
  ModelState update(const ModelState &state);
private:
  WindAndWavesParams p;
  std::default_random_engine gen;
  std::normal_distribution<double> dist_wF, dist_wN, dist_dF, dist_dN;

  double xN1{0}, xN2{0};
  double xF1{0}, xF2{0};
  double dF{0}, dN{0};
  double xF1_dot_last{0}, xF2_dot_last{0};
  double xN1_dot_last{0}, xN2_dot_last{0};
  double dF_dot_last{0}, dN_dot_last{0};

  constexpr static double w0 = 0.8;
  constexpr static double lambda = 0.1;
  constexpr static double Kw = 0.64;
  constexpr static double g = 9.81;

  constexpr static double rho = 1.0;
  constexpr static double CDlaf_0 = 0.55;
  constexpr static double CDlaf_pi = 0.6;
  constexpr static double CDt = 0.9;
  constexpr static double AFW = 0.045;
  constexpr static double ALW = 0.09;
  constexpr static double LOA = 0.9;
  constexpr static double delta_wind = 0.6;
};