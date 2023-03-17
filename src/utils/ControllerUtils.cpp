#include "ControllerUtils.h"

std::pair<DynamicModelOutput, ASMCOutput> ControllerUtils::update(DynamicModel &model, ASMC &controller, const ASMCSetpoint &setpoint) {
  auto state = fromModel(model);
  auto asmcOut = controller.update(state, setpoint);
  auto modelOut = model.update(asmcOut.left_thruster, asmcOut.right_thruster);
  return {modelOut, asmcOut};
}

std::pair<std::vector<DynamicModelOutput>, std::vector<ASMCOutput>>
ControllerUtils::update_n(DynamicModel &model, ASMC &controller, ASMCSetpoint setpoint, int n) {
  // Use heading setpoint struct as angular vel, use velocity as acceleration TESTING TESTING
  double angularVel = setpoint.heading_setpoint / static_cast<double>(n);
  setpoint.heading_setpoint = model.currentState().pose_psi;

  double acceleration = setpoint.velocity_setpoint / static_cast<double>(n);
  setpoint.velocity_setpoint = model.currentState().vel_x;

  std::vector<DynamicModelOutput> modelOutput(n);
  std::vector<ASMCOutput> asmcOutput(n);

  for(int i = 0; i < n; i++){
    // Update theta by angular vel
    setpoint.heading_setpoint += angularVel;
    //setpoint.heading_setpoint = ASMC::constrainAngle(setpoint.heading_setpoint);
    setpoint.velocity_setpoint += acceleration;
    setpoint.velocity_setpoint = std::clamp(setpoint.velocity_setpoint, 0.2, 0.75);
    std::tie(modelOutput[i], asmcOutput[i]) = update(model, controller, setpoint);
  }

  return {modelOutput, asmcOutput};
}

ASMCState ControllerUtils::fromModel(const DynamicModel &model) {
  const auto &state(model.currentState());
  ASMCState asmcState{};
  asmcState.vel_x = state.vel_x;
  asmcState.vel_y = state.vel_y;
  asmcState.vel_r = state.vel_r;
  asmcState.theta = state.pose_psi;
  return asmcState;
}