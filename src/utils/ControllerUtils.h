#pragma once
#include "control/ASMC.h"
#include "model/dynamic_model.h"

namespace ControllerUtils {
  // Updates controller with dynamic model, for use in simulation
  std::pair<DynamicModelOutput, ASMCOutput> update(DynamicModel &model, ASMC &controller, ASMCSetpoint &setpoint);
  ASMCState fromModel(const DynamicModel &model);
}