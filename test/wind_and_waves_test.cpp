#include <gtest/gtest.h>
#include "perturbances/wind_and_waves.h"

class WindAndWavesTest : public ::testing::Test {
protected:
  void SetUp() override {
    waves = WindAndWaves();
  }
  WindAndWaves waves;
};

TEST_F(WindAndWavesTest, SanityCheck){
  //Check if wind and waves returns something ne 0
  ModelState state;
  state.u = 1;

  ModelState perturb = waves.update(state);
  EXPECT_NE(perturb.pose_x, 0);
  EXPECT_NE(perturb.pose_y, 0);
  EXPECT_NE(perturb.pose_psi, 0);
}