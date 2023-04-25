//
// Created by Abiel on 4/24/23.
//

#include <gtest/gtest.h>
#include "model/dynamic_model.h"
#include "control/AITSMC.h"
#include "utils/ControllerUtils.h"
#include <iostream>

class AITSMCTest : public ::testing::Test {
protected:
  void SetUp() override {
    model = DynamicModel();
    controller = AITSMC(AITSMC::defaultParams());
  }

  DynamicModel model;
  AITSMC controller{AITSMC::defaultParams()};
  const int n = 10000;
};

TEST_F(AITSMCTest, ReachesVelocityAndHeading){
  AITSMCSetpoint setpoint{};
  setpoint.u = 0.1;
  setpoint.r = 0.5;

  for(int i = 0; i < n; i++){
    auto state = ControllerUtils::fromModel(model);
    auto out = controller.update(state, setpoint);
    model.update(out.left_thruster, out.right_thruster);
    auto debug = controller.getDebugData();
  }

  EXPECT_NEAR(model.currentState().u, setpoint.u, 0.1);
  EXPECT_NEAR(model.currentState().r, setpoint.r, 0.1);
}