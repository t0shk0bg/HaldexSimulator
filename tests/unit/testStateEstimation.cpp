#include "core/haldexController.hpp"

#include <gtest/gtest.h>

class StateEstimationTest : public ::testing::Test {
  protected:
    void SetUp() override {
        setupHaldexControl({HaldexControlConfig{}});
        setupVehiclePhysics(VehiclePhysicsConfig{});
        stateEstimationLayer = StateEstimationLayer{};
        processedSignalsLayer = SignalProcessingLayer{};
        filterState = FilterState{};
        rawCanInput = CanInputLayer{};
    }
};

// gear = 0 (Neutral) → expected torque is 0 Nm
TEST_F(StateEstimationTest, WheelTorqueZeroInNeutral) {
    rawCanInput.gear = 0;
    rawCanInput.actualTorqueSumNm = 300.0f;
    estimateWheelTorque(rawCanInput, physConfig, stateEstimationLayer);
    EXPECT_FLOAT_EQ(stateEstimationLayer.anticipatedWheelTorqueNm, 0.0f);
}

// gear 1, 400 Nm engine → 400 * ratio[1]=3.563 * fd1=4.059 * eff=0.91 ≈ 5264 Nm
TEST_F(StateEstimationTest, WheelTorqueGear1At400Nm) {
    rawCanInput.gear = 1;
    rawCanInput.actualTorqueSumNm = 400.0f;
    estimateWheelTorque(rawCanInput, physConfig, stateEstimationLayer);
    // 400 * 3.563 * 4.059 * 0.91 ≈ 5264 Nm
    EXPECT_NEAR(stateEstimationLayer.anticipatedWheelTorqueNm, 5264.0f, 20.0f);
}

// V < 0.55 m/s → both slip components are 0 regardless of the wheels
TEST_F(StateEstimationTest, ReactiveSlipZeroAtLowSpeed) {
    processedSignalsLayer.wheelSpeedsMps[0] = 5.0f;
    processedSignalsLayer.wheelSpeedsMps[1] = 5.0f;
    processedSignalsLayer.wheelSpeedsMps[2] = 3.0f;
    processedSignalsLayer.wheelSpeedsMps[3] = 3.0f;
    for (int i = 0; i < 4; i++) {
        processedSignalsLayer.wheelValid[i] = true;
    }

    estimateReactiveSlip(rawCanInput, processedSignalsLayer, stateEstimationLayer, 0.4f);
    EXPECT_FLOAT_EQ(stateEstimationLayer.frontRearSlipMps, 0.0f);
    EXPECT_FLOAT_EQ(stateEstimationLayer.rearOverrunSlipMps, 0.0f);
}

// front > rear → frontRearSlip > 0 and rearOverrun = 0; the reverse gives rearOverrun > 0
TEST_F(StateEstimationTest, ReactiveSlipDirectionality) {
    for (int i = 0; i < 4; i++) {
        processedSignalsLayer.wheelValid[i] = true;
    }

    // Front faster than rear → frontRearSlip > 0
    processedSignalsLayer.wheelSpeedsMps[0] = 5.0f;
    processedSignalsLayer.wheelSpeedsMps[1] = 5.0f;
    processedSignalsLayer.wheelSpeedsMps[2] = 4.0f;
    processedSignalsLayer.wheelSpeedsMps[3] = 4.0f;
    estimateReactiveSlip(rawCanInput, processedSignalsLayer, stateEstimationLayer, 2.0f);
    EXPECT_GT(stateEstimationLayer.frontRearSlipMps, 0.0f);
    EXPECT_FLOAT_EQ(stateEstimationLayer.rearOverrunSlipMps, 0.0f);

    // Rear faster than front → rearOverrunSlip > 0
    processedSignalsLayer.wheelSpeedsMps[0] = 4.0f;
    processedSignalsLayer.wheelSpeedsMps[1] = 4.0f;
    processedSignalsLayer.wheelSpeedsMps[2] = 5.0f;
    processedSignalsLayer.wheelSpeedsMps[3] = 5.0f;
    estimateReactiveSlip(rawCanInput, processedSignalsLayer, stateEstimationLayer, 2.0f);
    EXPECT_FLOAT_EQ(stateEstimationLayer.frontRearSlipMps, 0.0f);
    EXPECT_GT(stateEstimationLayer.rearOverrunSlipMps, 0.0f);
}

// When escOff = true → tireGrip = 1.0 regardless of how much G load there is
TEST_F(StateEstimationTest, TireGripAlways1WhenEscOff) {
    rawCanInput.escOff = true;
    rawCanInput.lateralAccelG = 0.9f;
    rawCanInput.longitudinalAccelG = 0.8f;
    estimateLateralDynamics(rawCanInput, physConfig, processedSignalsLayer, stateEstimationLayer, 15.0f);
    EXPECT_FLOAT_EQ(stateEstimationLayer.tireGripAvailablePct, 1.0f);
}

// When escOff = false with combined lat + long G → the friction ellipse reduces grip < 1.0
TEST_F(StateEstimationTest, FrictionEllipseReducesGripUnderCombinedLoad) {
    rawCanInput.escOff = false;
    rawCanInput.lateralAccelG = 0.5f;
    rawCanInput.longitudinalAccelG = 0.5f;
    // longN = 0.5/0.95 = 0.526, latN = 0.5/1.05 = 0.476
    // grip = sqrt(1 - 0.526^2 - 0.476^2) = sqrt(0.497) ≈ 0.705
    estimateLateralDynamics(rawCanInput, physConfig, processedSignalsLayer, stateEstimationLayer, 15.0f);
    EXPECT_LT(stateEstimationLayer.tireGripAvailablePct, 1.0f);
    EXPECT_NEAR(stateEstimationLayer.tireGripAvailablePct, 0.705f, 0.02f);
}

// On trail braking at the limit (0.95G + 1.05G) → tireGripAvailablePct = 0.0 (real physics)
// The predictive lock floor is applied separately during scaling at line 410 — not here
TEST_F(StateEstimationTest, TireGripIsZeroAtExactEllipseLimit) {
    rawCanInput.escOff = false;
    rawCanInput.longitudinalAccelG = 0.95f; // = tireMaxLongG
    rawCanInput.lateralAccelG = 1.05f;      // = tireMaxLateralG
    // longN = 1.0, latN = 1.0 → gripSquared = -1.0 → grip = 0.0
    estimateLateralDynamics(rawCanInput, physConfig, processedSignalsLayer, stateEstimationLayer, 15.0f);
    EXPECT_FLOAT_EQ(stateEstimationLayer.tireGripAvailablePct, 0.0f);
}

// reverseGear = true → anticipatedWheelTorqueNm = 0.0, regardless of throttle
TEST_F(StateEstimationTest, WheelTorqueZeroWhenReverseGearSet) {
    rawCanInput.reverseGear = true;
    rawCanInput.gear = 0;
    rawCanInput.actualTorqueSumNm = 300.0f;
    estimateWheelTorque(rawCanInput, physConfig, stateEstimationLayer);
    EXPECT_FLOAT_EQ(stateEstimationLayer.anticipatedWheelTorqueNm, 0.0f);
}

// gear > 7 (invalid value) → anticipatedWheelTorqueNm = 0.0 (defensive guard)
TEST_F(StateEstimationTest, WheelTorqueZeroForOutOfBoundsGear) {
    rawCanInput.gear = 8;
    rawCanInput.actualTorqueSumNm = 300.0f;
    estimateWheelTorque(rawCanInput, physConfig, stateEstimationLayer);
    EXPECT_FLOAT_EQ(stateEstimationLayer.anticipatedWheelTorqueNm, 0.0f);
}

// Regression test: a mirrored scenario (left vs right) yields the same deviation.
// Before the fix: right ≈ -0.064, left ≈ -1.064 (asymmetry due to a missing abs()).
// After the fix: both ≈ -0.064 (symmetric).
TEST_F(StateEstimationTest, ChassisSlipDeviationIsSymmetricLeftRight) {
    processedSignalsLayer.steeringAngleRad = 0.3f;
    processedSignalsLayer.yawRateRadS = 0.5f;
    filterState.laggedExpectedYawRadS = 0.5f;
    estimateSteeringAndYaw(processedSignalsLayer, filterState,
                           stateEstimationLayer, 15.0f, 0.01f);
    float deviationRight = stateEstimationLayer.chassisSlipDeviationRadS;

    stateEstimationLayer = StateEstimationLayer{};
    filterState = FilterState{};
    processedSignalsLayer = SignalProcessingLayer{};

    processedSignalsLayer.steeringAngleRad = -0.3f;
    processedSignalsLayer.yawRateRadS = -0.5f;
    filterState.laggedExpectedYawRadS = -0.5f;
    estimateSteeringAndYaw(processedSignalsLayer, filterState,
                           stateEstimationLayer, 15.0f, 0.01f);
    float deviationLeft = stateEstimationLayer.chassisSlipDeviationRadS;

    EXPECT_NEAR(deviationRight, deviationLeft, 0.001f);
}

// Regression: validateChassisResponse must see the CURRENT-cycle oversteer
// (fresh chassisSlipDeviationRadS), not the previous cycle's value.
// Before the fix the deviation was read one cycle late: with a zeroed filterState
// the stale value was 0 (not oversteer), so the slalom branch failed upperBound and
// cornerEntryPredicted came out false. With the fix the fresh oversteer is seen,
// the escOff oversteer branch returns directionOk, and cornerEntryPredicted is true.
TEST_F(StateEstimationTest, OversteerDetectionUsesCurrentCycleDeviation) {
    rawCanInput.escOff = true;
    processedSignalsLayer.steeringAngleRad = 0.3f; // → steeringTrigger + slalom transition
    processedSignalsLayer.yawRateRadS = 0.5f;      // real yaw >> lagged expected → oversteer this cycle
    // filterState starts zeroed → previous-cycle chassisSlipDeviationRadS = 0 (NOT oversteer)
    estimateSteeringAndYaw(processedSignalsLayer, filterState, stateEstimationLayer, 15.0f, 0.01f);

    EXPECT_LT(stateEstimationLayer.chassisSlipDeviationRadS, -0.0698f); // sanity: oversteer this cycle
    EXPECT_TRUE(stateEstimationLayer.cornerEntryPredicted);            // would be false on the stale read
}
