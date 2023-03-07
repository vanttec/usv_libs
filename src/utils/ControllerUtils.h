#pragma once
#include "control/ASMC.h"
#include "model/dynamic_model.h"

namespace ControllerUtils {
  // Updates controller with dynamic model, for use in simulation
  std::pair<DynamicModelOutput, ASMCOutput> update(DynamicModel &model, ASMC &controller, const ASMCSetpoint &setpoint);
  ASMCState fromModel(const DynamicModel &model);

  // Used to run asmc and model n times, speeds up python bindings
  std::pair<DynamicModelOutput, ASMCOutput> update_n(DynamicModel &model, ASMC &controller, const ASMCSetpoint &setpoint, int n);
}