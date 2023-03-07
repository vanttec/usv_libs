#include "ControllerUtils.h"

std::pair<DynamicModelOutput, ASMCOutput> ControllerUtils::update(DynamicModel &model, ASMC &controller, ASMCSetpoint &setpoint) {
  auto state = fromModel(model);
  auto asmcOut = controller.update(state, setpoint);
  auto modelOut = model.update(asmcOut.left_thruster, asmcOut.right_thruster);
  return {modelOut, asmcOut};
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