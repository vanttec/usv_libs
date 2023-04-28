#include "ControllerUtils.h"

std::pair<ModelState, ASMCOutput> ControllerUtils::update(DynamicModel &model, ASMC &controller, const ASMCSetpoint &setpoint) {
  auto state = fromModelASMC(model);
  auto asmcOut = controller.update(state, setpoint);
  auto modelOut = model.update(asmcOut.left_thruster, asmcOut.right_thruster);
  return {modelOut, asmcOut};
}

std::pair<std::vector<ModelState>, std::vector<ASMCOutput>>
ControllerUtils::update_n(DynamicModel &model, ASMC &controller, ASMCSetpoint setpoint, int n) {
  // Use heading setpoint struct as angular vel, use velocity as acceleration TESTING TESTING
  double angularVel = setpoint.heading_setpoint / static_cast<double>(n);
  setpoint.heading_setpoint = model.currentState().pose_psi;

  double acceleration = setpoint.velocity_setpoint / static_cast<double>(n);
  setpoint.velocity_setpoint = model.currentState().u;

  std::vector<ModelState> modelOutput(n);
  std::vector<ASMCOutput> asmcOutput(n);

  for(int i = 0; i < n; i++){
    // Update theta by angular vel
    auto state = model.currentState();
    setpoint.heading_setpoint = state.pose_psi + angularVel;
    //setpoint.heading_setpoint = ASMC::constrainAngle(setpoint.heading_setpoint);
    setpoint.velocity_setpoint = state.u + acceleration;
    setpoint.velocity_setpoint = std::clamp(setpoint.velocity_setpoint, 0.5, 0.8);
    std::tie(modelOutput[i], asmcOutput[i]) = update(model, controller, setpoint);
  }

  return {modelOutput, asmcOutput};
}

ASMCState ControllerUtils::fromModelASMC(const DynamicModel &model) {
  const auto &state(model.currentState());
  ASMCState asmcState{};
  asmcState.vel_x = state.u;
  asmcState.vel_y = state.v;
  asmcState.vel_r = state.r;
  asmcState.theta = state.pose_psi;
  return asmcState;
}

vanttec::ControllerState ControllerUtils::fromModel(const DynamicModel &model) {
  const auto &s(model.currentState());
  vanttec::ControllerState state{};
  state.u = s.u;
  state.v = s.v;
  state.r = s.r;
  state.psi = s.pose_psi;
  return state;
}