#include "ControllerUtils.h"

std::pair<DynamicModelOutput, ASMCOutput> ControllerUtils::update(DynamicModel &model, ASMC &controller, const ASMCSetpoint &setpoint) {
  auto state = fromModel(model);
  auto asmcOut = controller.update(state, setpoint);
  auto modelOut = model.update(asmcOut.left_thruster, asmcOut.right_thruster);
  return {modelOut, asmcOut};
}

std::pair<DynamicModelOutput, ASMCOutput>
ControllerUtils::update_n(DynamicModel &model, ASMC &controller, const ASMCSetpoint &setpoint, int n) {
  std::pair<DynamicModelOutput, ASMCOutput> out;
  for(int i = 0; i < n; i++){
    out = update(model, controller, setpoint);
  }

  return out;
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