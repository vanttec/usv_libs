#include <gtest/gtest.h>
#include "model/dynamic_model.h"
#include "control/ASMC.h"
#include "utils/ControllerUtils.h"

class ASMCTest : public ::testing::Test {
protected:
  void SetUp() override{
    model = DynamicModel();
    controller = ASMC(ASMC::defaultParams());
  }

  DynamicModel model{};
  ASMC controller{ASMC::defaultParams()};
  const int n = 1000;
};

TEST_F(ASMCTest, ReachesVelocityAndHeading){
  auto modelState = model.update(0, 0);
  ASMCOutput out{};
  ASMCSetpoint setpoint{};
  setpoint.heading_setpoint = M_PI_2;
  setpoint.velocity_setpoint = 0.5;
  for(int i = 0; i < n; i++){
    std::tie(modelState, out) = ControllerUtils::update(model, controller, setpoint);
  }

  EXPECT_NEAR(modelState.vel_x, setpoint.velocity_setpoint, 0.1);
  EXPECT_NEAR(modelState.pose_psi, setpoint.heading_setpoint, 0.5);
}
