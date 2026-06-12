#include "core/haldexController.hpp"

#include <cmath>
#include <gtest/gtest.h>

// Steering ratio used to convert the tire angle in the tests into a column angle (config default).
static constexpr float kTestSteeringRatio = 14.5f;

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
    estimateWheelTorque(rawCanInput, stateEstimationLayer);
    EXPECT_FLOAT_EQ(stateEstimationLayer.anticipatedWheelTorqueNm, 0.0f);
}

// gear 1, 400 Nm engine → 400 * ratio[1]=3.563 * fd1=4.059 * eff=0.91 ≈ 5264 Nm
TEST_F(StateEstimationTest, WheelTorqueGear1At400Nm) {
    rawCanInput.gear = 1;
    rawCanInput.actualTorqueSumNm = 400.0f;
    estimateWheelTorque(rawCanInput, stateEstimationLayer);
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

// Helper: set all four wheels valid and assign front/rear pair speeds [m/s].
static void setWheels(float front, float rear) {
    for (int i = 0; i < 4; i++) {
        processedSignalsLayer.wheelValid[i] = true;
    }
    processedSignalsLayer.wheelSpeedsMps[0] = front;
    processedSignalsLayer.wheelSpeedsMps[1] = front;
    processedSignalsLayer.wheelSpeedsMps[2] = rear;
    processedSignalsLayer.wheelSpeedsMps[3] = rear;
}

// Pure geometric turn (front = rear / cos δ, perfect grip): compensation removes it → slip ≈ 0.
TEST_F(StateEstimationTest, SlipCompensatedAwayInDryTurn) {
    rawCanInput.escOff = true; // wheelDataConfidence = 1.0, isolate compensation
    float tireAngle = 0.40f;
    processedSignalsLayer.steeringAngleRad = tireAngle * kTestSteeringRatio;
    float rear = 10.0f;
    setWheels(rear / std::cos(tireAngle), rear);
    // kinematicYawConfidence defaults to 1.0 → full geometric compensation
    estimateReactiveSlip(rawCanInput, processedSignalsLayer, stateEstimationLayer, 12.0f);
    EXPECT_NEAR(stateEstimationLayer.frontRearSlipMps, 0.0f, 0.01f);
}

// Real slip on top of the geometry survives compensation.
TEST_F(StateEstimationTest, RealSlipSurvivesCompensation) {
    rawCanInput.escOff = true;
    float tireAngle = 0.40f;
    processedSignalsLayer.steeringAngleRad = tireAngle * kTestSteeringRatio;
    float rear = 10.0f;
    setWheels(rear / std::cos(tireAngle) + 1.0f, rear); // geometry + 1 m/s genuine slip
    estimateReactiveSlip(rawCanInput, processedSignalsLayer, stateEstimationLayer, 12.0f);
    EXPECT_NEAR(stateEstimationLayer.frontRearSlipMps, 1.0f, 0.02f);
}

// Symmetric compensation: a pure geometric turn leaves rearOverrun at exactly 0 too.
TEST_F(StateEstimationTest, RearOverrunZeroInPureGeometricTurn) {
    rawCanInput.escOff = true;
    float tireAngle = 0.40f;
    processedSignalsLayer.steeringAngleRad = tireAngle * kTestSteeringRatio;
    float rear = 10.0f;
    setWheels(rear / std::cos(tireAngle), rear);
    estimateReactiveSlip(rawCanInput, processedSignalsLayer, stateEstimationLayer, 12.0f);
    EXPECT_NEAR(stateEstimationLayer.frontRearSlipMps, 0.0f, 0.01f);
    EXPECT_NEAR(stateEstimationLayer.rearOverrunSlipMps, 0.0f, 0.01f);
}

// Lower confidence fades the geometric subtraction → more residual slip is detected.
TEST_F(StateEstimationTest, B2FadesCompensationUnderUndersteer) {
    rawCanInput.escOff = true;
    float tireAngle = 0.40f;
    processedSignalsLayer.steeringAngleRad = tireAngle * kTestSteeringRatio;
    float rear = 10.0f;
    setWheels(rear / std::cos(tireAngle), rear); // pure geometry

    stateEstimationLayer.kinematicYawConfidence = 1.0f;
    estimateReactiveSlip(rawCanInput, processedSignalsLayer, stateEstimationLayer, 12.0f);
    float slipFull = stateEstimationLayer.frontRearSlipMps;

    stateEstimationLayer.kinematicYawConfidence = 0.5f; // understeer fade
    estimateReactiveSlip(rawCanInput, processedSignalsLayer, stateEstimationLayer, 12.0f);
    float slipFaded = stateEstimationLayer.frontRearSlipMps;

    EXPECT_GT(slipFaded, slipFull + 0.05f);
}

// Opposite-sign yaw (spin/countersteer) drives the signed confidence to the floor → raw fallback.
TEST_F(StateEstimationTest, CompensationFallsBackOnOppositeYaw) {
    processedSignalsLayer.steeringAngleRad = 3.0f; // positive → expected yaw positive
    processedSignalsLayer.yawRateRadS = -0.8f;     // car yaws the other way
    filterState.laggedExpectedYawRadS = 0.5f;      // positive lagged reference
    estimateSteeringAndYaw(processedSignalsLayer, filterState, stateEstimationLayer, 15.0f, 0.01f);
    EXPECT_FLOAT_EQ(stateEstimationLayer.kinematicYawConfidence, activeConfig().slipCompYawConfidenceFloor);
}

// Below the chassis-dynamics speed the lagged yaw is frozen → confidence is forced to 1.0.
TEST_F(StateEstimationTest, ConfidenceOneAtLowSpeed) {
    processedSignalsLayer.steeringAngleRad = 3.0f;
    processedSignalsLayer.yawRateRadS = -0.8f; // would yield the floor if it were computed
    filterState.laggedExpectedYawRadS = 0.5f;
    estimateSteeringAndYaw(processedSignalsLayer, filterState, stateEstimationLayer, 3.0f, 0.01f); // V < 4.16
    EXPECT_FLOAT_EQ(stateEstimationLayer.kinematicYawConfidence, 1.0f);
}

// Master switch off → legacy raw avgFront - avgRear (geometry NOT removed).
TEST_F(StateEstimationTest, CompensationDisabledIsLegacy) {
    HaldexControlConfig cfg;
    cfg.steeringSlipCompensationEnabled = false;
    setupHaldexControl({cfg});

    rawCanInput.escOff = true;
    float tireAngle = 0.40f;
    processedSignalsLayer.steeringAngleRad = tireAngle * kTestSteeringRatio;
    float rear = 10.0f;
    float front = rear / std::cos(tireAngle);
    setWheels(front, rear);
    estimateReactiveSlip(rawCanInput, processedSignalsLayer, stateEstimationLayer, 12.0f);
    EXPECT_NEAR(stateEstimationLayer.frontRearSlipMps, front - rear, 0.02f);
}

// Regression: the raised slipReactiveGain default (14) flows into dynamicSlipGain at zero load.
TEST_F(StateEstimationTest, DynamicSlipGainReflectsRaisedDefault) {
    setWheels(10.0f, 10.0f);
    rawCanInput.longitudinalAccelG = 0.0f; // loadTransferMultiplier = 1.0
    estimateReactiveSlip(rawCanInput, processedSignalsLayer, stateEstimationLayer, 12.0f);
    EXPECT_NEAR(stateEstimationLayer.dynamicSlipGain, 14.0f, 0.01f);
}

// When escOff = true → tireGrip = 1.0 regardless of how much G load there is
TEST_F(StateEstimationTest, TireGripAlways1WhenEscOff) {
    rawCanInput.escOff = true;
    rawCanInput.lateralAccelG = 0.9f;
    rawCanInput.longitudinalAccelG = 0.8f;
    estimateLateralDynamics(rawCanInput, processedSignalsLayer, stateEstimationLayer, 15.0f);
    EXPECT_FLOAT_EQ(stateEstimationLayer.tireGripAvailablePct, 1.0f);
}

// When escOff = false with combined lat + long G → the friction ellipse reduces grip < 1.0
TEST_F(StateEstimationTest, FrictionEllipseReducesGripUnderCombinedLoad) {
    rawCanInput.escOff = false;
    rawCanInput.lateralAccelG = 0.5f;
    rawCanInput.longitudinalAccelG = 0.5f;
    // longN = 0.5/0.95 = 0.526, latN = 0.5/1.05 = 0.476
    // grip = sqrt(1 - 0.526^2 - 0.476^2) = sqrt(0.497) ≈ 0.705
    estimateLateralDynamics(rawCanInput, processedSignalsLayer, stateEstimationLayer, 15.0f);
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
    estimateLateralDynamics(rawCanInput, processedSignalsLayer, stateEstimationLayer, 15.0f);
    EXPECT_FLOAT_EQ(stateEstimationLayer.tireGripAvailablePct, 0.0f);
}

// reverseGear = true → anticipatedWheelTorqueNm = 0.0, regardless of throttle
TEST_F(StateEstimationTest, WheelTorqueZeroWhenReverseGearSet) {
    rawCanInput.reverseGear = true;
    rawCanInput.gear = 0;
    rawCanInput.actualTorqueSumNm = 300.0f;
    estimateWheelTorque(rawCanInput, stateEstimationLayer);
    EXPECT_FLOAT_EQ(stateEstimationLayer.anticipatedWheelTorqueNm, 0.0f);
}

// gear > 7 (invalid value) → anticipatedWheelTorqueNm = 0.0 (defensive guard)
TEST_F(StateEstimationTest, WheelTorqueZeroForOutOfBoundsGear) {
    rawCanInput.gear = 8;
    rawCanInput.actualTorqueSumNm = 300.0f;
    estimateWheelTorque(rawCanInput, stateEstimationLayer);
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
