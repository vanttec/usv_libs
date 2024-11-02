#include "uuv_dynamic_model.h"

UUVDynamicModel::UUVDynamicModel(Eigen::VectorXf eta) {
  state.eta_dot = Eigen::VectorXf::Zero(6);
  state.eta_dot_prev = Eigen::VectorXf::Zero(6);
  state.nu = Eigen::VectorXf::Zero(6);
  state.nu_dot = Eigen::VectorXf::Zero(6);
  state.nu_dot_prev = Eigen::VectorXf::Zero(6);
  UUVDynamicModel();
  state.eta = eta;
}

UUVDynamicModel::UUVDynamicModel() {
  std::cout << "init" << std::endl;
  state.eta = Eigen::VectorXf::Zero(6);
  state.eta_dot = Eigen::VectorXf::Zero(6);
  state.eta_dot_prev = Eigen::VectorXf::Zero(6);
  state.nu = Eigen::VectorXf::Zero(6);
  state.nu_dot = Eigen::VectorXf::Zero(6);
  state.nu_dot_prev = Eigen::VectorXf::Zero(6);

  J_ = Eigen::MatrixXf::Zero(6, 6);
  R_ = Eigen::Matrix3f::Zero(3,3);
  T_ = Eigen::Matrix3f::Zero(3,3);

  f_x_ = Eigen::MatrixXf::Zero(6,1);
  g_x_ = Eigen::MatrixXf::Zero(6,6);

  M_ = Eigen::MatrixXf::Identity(6,6);
  Eigen::VectorXf m_diag; 
  m_diag = Eigen::VectorXf::Zero(6);
  m_diag << W_-X_u_dot_,W_-Y_v_dot_,W_-Z_w_dot_,Ixx_-K_p_dot_,Iyy_-M_q_dot_,Izz_-N_r_dot_;
  M_ = m_diag.asDiagonal();

  // Eigen::DiagonalMatrix m_diag(W_-X_u_dot_,W_-Y_v_dot_,W_-Z_w_dot_,Ixx_-K_p_dot_,Iyy_-M_q_dot_,Izz_-N_r_dot_)
  // M_.dot(m_diag);
  C_ = Eigen::MatrixXf::Zero(6, 6);
  D_ = Eigen::MatrixXf::Zero(6, 6);
  G_ = Eigen::MatrixXf::Zero(6, 1);
  u_ = Eigen::MatrixXf::Zero(6, 1);
}

/*
  M_ << 
    W_-X_u_dot_, 0, 0, 0, 0, 0,
    0, W_-Y_v_dot_, 0, 0, 0, 0,
    0, 0, W_-Z_w_dot_, 0, 0, 0,
    0, 0, 0, Ixx_-K_p_dot_, 0, 0,
    0, 0, 0, 0, Iyy_-M_q_dot_, 0,
    0, 0, 0, 0, 0, Izz_-N_r_dot_;
*/

UUVState UUVDynamicModel::update(std::array<double, 6> t) {
  UUVState out;
  out = state;
  u_ << t[0], t[1], t[2], t[3], t[4], t[5];
  std::cout << "antes de todo nu: \n" << out.nu << std::endl;
  matricesUpdate();
  g_x_ = M_.inverse();
  f_x_ = -M_.inverse() * (C_ * out.nu + D_ * out.nu + G_);  

  out.nu_dot = f_x_ + g_x_*(u_);
  std::cout << "\nantes: \n" << out.nu << "\ndeberia de: \n" << dt * (out.nu_dot + out.nu_dot_prev) / 2 << std::endl;
  out.nu = dt * (out.nu_dot + out.nu_dot_prev) / 2;
  out.nu_dot_prev = out.nu_dot;

  out.eta_dot = J_ * out.nu;
  out.eta =  dt * (out.eta_dot + out.eta_dot_prev) / 2;
  out.eta_dot_prev = out.eta_dot;

  std::cout << "\ndespues nu: \n" << out.nu << "\neta: \n" << out.eta << std::endl;
  state = out;
  return out;
}

double UUVDynamicModel::constrainAngle(double angle) {
  angle = std::fmod(angle + M_PI, 2 * M_PI);
  if(angle < 0){
    angle+=2*M_PI;
  }
  return angle -M_PI;
}

void UUVDynamicModel::matricesUpdate(){
  double u,v,w,p,q,r,phi,theta, psi;
  u = state.nu(0);
  v = state.nu(1);
  w = state.nu(2);
  p = state.nu(3);
  q = state.nu(4);
  r = state.nu(5);
  phi = state.eta(3);
  theta = state.eta(4);
  psi = state.eta(5);

  R_ <<
    std::cos(psi)*std::cos(theta),
    -std::sin(psi)*std::cos(phi) + std::cos(psi)*std::sin(theta)*std::sin(phi),
    std::sin(psi)*std::sin(phi) + std::cos(psi)*std::cos(phi)*std::sin(theta),
    std::sin(psi)*std::cos(theta),
    std::cos(psi)*std::cos(phi)+std::sin(phi)*std::sin(theta)*std::sin(psi),
    -std::cos(psi)*std::sin(phi)+std::sin(theta)*std::sin(psi)*std::cos(phi),
    -std::sin(theta),
    std::cos(theta)*std::sin(phi),
    std::cos(theta)*std::cos(phi)
  ;

  T_ <<
    1, std::sin(phi)*std::tan(theta), std::cos(phi)*std::tan(theta),
    0, std::cos(phi), -std::sin(phi),
    0, std::sin(phi)/std::cos(theta), std::cos(phi)/std::cos(theta)
  ;

  J_ = Eigen::MatrixXf::Zero(6, 6);
  J_.topLeftCorner(3,3) = R_;
  J_.bottomRightCorner(3,3) = T_;

  double a1, a2, a3, b1, b2, b3;
  a1 = W_*u - X_u_dot_*u;
  a2 = W_*v - Y_v_dot_*v;
  a3 = W_*w - Z_w_dot_*w;
  b1 = Ixx_*p + K_p_dot_*p;
  b2 = Iyy_*q + M_q_dot_*q;
  b3 = Izz_*r + N_r_dot_*r;
  C_ <<
    0, 0, 0, 0, a3, -a2,
    0, 0, 0, -a3, 0, a1,
    0, 0, 0, a2, -a1, 0,
    0, a3, -a2, 0, -b3, b2,
    -a3, 0, a1, b3, 0, -b1,
    a2, -a1, 0, -b2, b1, 0;

  D_ = Eigen::MatrixXf::Identity(6,6);
  Eigen::VectorXf d_diag; 
  d_diag = Eigen::VectorXf::Zero(6);
  d_diag << 
    X_u_+X_uu_*std::fabs(u),
    Y_v_+Y_vv_*std::fabs(v),
    Z_w_+Z_ww_*std::fabs(w),
    K_p_+K_pp_*std::fabs(p),
    M_q_+M_qq_*std::fabs(q),
    N_r_+N_rr_*std::fabs(r)
  ;
  D_ = d_diag.asDiagonal()*-1;

  // TODO: REVISAR ESTA DEFINICION
  G_ << 
    (W_-B_)*std::sin(theta),
    -(W_-B_)*std::cos(theta)*std::sin(phi),
    -(W_-B_)*std::cos(theta)*std::cos(phi),
    -rb_z_*B_*std::cos(theta)*std::sin(phi),
    -rb_z_*B_*std::sin(theta),
    0
  ;
}