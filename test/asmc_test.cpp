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
  const int n = 5000;
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

  EXPECT_NEAR(modelState.u, setpoint.velocity_setpoint, 0.1);
  EXPECT_NEAR(modelState.pose_psi, setpoint.heading_setpoint, 0.5);
}

TEST_F(ASMCTest, UpdateNReachesVelAndHeading){
  auto modelState = model.update(0, 0);
  ASMCOutput out{};
  ASMCSetpoint setpoint{};

  //Janky code, this is actuall ang vel, and accel
  setpoint.heading_setpoint = M_PI_2;
  setpoint.velocity_setpoint = 0.5;
  std::vector<ModelState> modelStates;
  std::vector<ASMCOutput> asmcOutputs;
  std::tie(modelStates, asmcOutputs) = ControllerUtils::update_n(model, controller, setpoint, n);

  EXPECT_GT(modelStates[n-1].u, 0);
  EXPECT_GT(modelStates[n-1].pose_psi, 0);
}
