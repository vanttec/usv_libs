#include <gtest/gtest.h>
#include "model/dynamic_model.h"

class DynamicModelTest : public ::testing::Test {
protected:
  void SetUp() override {
    model = DynamicModel();
  }

  DynamicModel model{};
  const int n = 100;
  const double tol = 0.1;
};

TEST_F(DynamicModelTest, StaticTest){
  //Boat should stay relatively still when no thruster output is applied
  DynamicModelOutput out{};
  for(int i = 0; i < n; i++){
    out = model.update(0, 0);
  }

  EXPECT_NEAR(out.pose_x, 0, tol);
  EXPECT_NEAR(out.pose_y, 0, tol);
  EXPECT_NEAR(out.pose_psi, 0, tol);

  EXPECT_NEAR(out.u, 0, tol);
  EXPECT_NEAR(out.v, 0, tol);
  EXPECT_NEAR(out.r, 0, tol);
}

TEST_F(DynamicModelTest, MoveForward){
  DynamicModelOutput out{};
  for(int i = 0; i < n; i++){
    out = model.update(5, 5);
  }

  EXPECT_NE(out.pose_x, 0);
  EXPECT_NEAR(out.pose_y, 0, tol);
  EXPECT_NEAR(out.pose_psi, 0, tol);

  EXPECT_NE(out.u, 0);
  EXPECT_NEAR(out.v, 0, tol);
  EXPECT_NEAR(out.r, 0, tol);
}