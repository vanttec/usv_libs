#pragma once
#include "control/ASMC.h"
#include "model/dynamic_model.h"
#include "control/datatypes.h"
#include <vector>

namespace ControllerUtils {
  // Updates controller with dynamic model, for use in simulation
  std::pair<DynamicModelOutput, ASMCOutput> update(DynamicModel &model, ASMC &controller, const ASMCSetpoint &setpoint);
  ASMCState fromModelASMC(const DynamicModel &model);
  vanttec::ControllerState fromModel(const DynamicModel &model);

  // Used to run asmc and model n times, speeds up python bindings
  // NOTE, setpoint heading is used as an angular velocity input :)
  std::pair<std::vector<DynamicModelOutput>, std::vector<ASMCOutput>> update_n(DynamicModel &model, ASMC &controller, ASMCSetpoint setpoint, int n);
}