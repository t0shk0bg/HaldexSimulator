#include "core/haldexController.hpp"

#include <gtest/gtest.h>

class LockCalculationTest : public ::testing::Test {
  protected:
    void SetUp() override {
        setupHaldexControl({HaldexControlConfig{}});
        setupVehiclePhysics(VehiclePhysicsConfig{});
        stateEstimationLayer = StateEstimationLayer{};
        rawCanInput = CanInputLayer{};
    }
};

// V >= cut_off (26.38 m/s / 95 km/h) → base lock = 0%
TEST_F(LockCalculationTest, BaseLockZeroAboveCutoff) {
    EXPECT_FLOAT_EQ(calculateBaseSpeedLock(26.38f), 0.0f);
    EXPECT_FLOAT_EQ(calculateBaseSpeedLock(40.0f), 0.0f);
}

// V = 0 → t = 0 → lock = baseLockLowSpeed * (1 - 0^exp) = 35%
TEST_F(LockCalculationTest, BaseLockMaxAtZeroSpeed) {
    EXPECT_NEAR(calculateBaseSpeedLock(0.0f), 35.0f, 0.01f);
}

// wheel_torque = 0 → normalized = 0 → torque lock = 0%
TEST_F(LockCalculationTest, TorqueLockZeroAtNoTorque) {
    EXPECT_FLOAT_EQ(calculateFeedforwardTorqueLock(0.0f), 0.0f);
}

// wheel_torque = maxWheelTorque → normalized = 1.0 → lock = throttleProactiveGain (60%)
TEST_F(LockCalculationTest, TorqueLockMaxAtFullTorque) {
    // physConfig.maxWheelTorque is computed by setupVehiclePhysics
    // maxWheelTorque = max(fd1Max, fd2Max) ≈ 5264 Nm
    EXPECT_NEAR(calculateFeedforwardTorqueLock(physConfig.maxWheelTorque), 60.0f, 0.01f);
}

// V >= steeringReductionLowSpeedThreshold (5.556 m/s) → factor = 1.0 (no degradation)
TEST_F(LockCalculationTest, ParkingDegradationInactiveAboveSpeedThreshold) {
    EXPECT_FLOAT_EQ(calculateParkingDegradation(5.556f, 0.0f), 1.0f);
    EXPECT_FLOAT_EQ(calculateParkingDegradation(20.0f, 1.5f), 1.0f);
}

// chassisSlipDeviation > understeerThreshold (0.0523 rad/s) → positive correction (adds lock)
TEST_F(LockCalculationTest, ChassisBalanceUndersteerAddsLock) {
    // deviation = 0.2 rad/s, threshold = 0.0523
    // raw_delta = 0.1477 rad/s → 8.46 deg/s → gain=2.0 → +16.93%, capped to 30%
    const HaldexControlConfig& cfg = activeConfig();
    float adjustment = calculateLateralChassisBalanceAdjustment(
        0.2f, false, cfg, 0.0f);
    EXPECT_GT(adjustment, 0.0f);
    EXPECT_LE(adjustment, cfg.balanceUndersteerMaxLock);
}

// slipDeviation = -0.12 rad/s (< -0.0698 threshold) → oversteer active
TEST_F(LockCalculationTest, IsOversteerActiveWhenBelowThreshold) {
    EXPECT_TRUE(isOversteerActive(-0.12f));
}

// slipDeviation exactly at the threshold (-0.0698) → < not satisfied → false
// slipDeviation above the threshold (+0.02) → false
TEST_F(LockCalculationTest, IsOversteerActiveFalseAtOrAboveThreshold) {
    EXPECT_FALSE(isOversteerActive(activeConfig().balanceOversteerThresholdRadS));
    EXPECT_FALSE(isOversteerActive(0.02f));
}

// oversteer + escOff + cornerEntryPredicted → boost must be 0
// Conditions: V=30 m/s (> cut-off) → baseSpeedLock=0; torque=0 → torqueLock=0; lateralFeedback=0
// Without fix: lateralDynamicLock = 0 + 0 + 25*0.5 = 12.5%
// With fix:    lateralDynamicLock = 0 + 0 + 0       = 0%
TEST_F(LockCalculationTest, CornerEntryBoostSuppressedDuringOversteerEscOff) {
    stateEstimationLayer.chassisSlipDeviationRadS = -0.12f;
    stateEstimationLayer.cornerEntryPredicted = true;
    stateEstimationLayer.stabilityMarginPct = 0.5f;
    stateEstimationLayer.lateralFeedbackLock = 0.0f;
    stateEstimationLayer.anticipatedWheelTorqueNm = 0.0f;
    rawCanInput.escOff = true;

    float result = calculatePredictiveLocks(30.0f);
    EXPECT_FLOAT_EQ(result, 0.0f);
}

// Without oversteer → boost must be active (25% × 0.5 = 12.5%)
TEST_F(LockCalculationTest, CornerEntryBoostActiveWithoutOversteer) {
    stateEstimationLayer.chassisSlipDeviationRadS = 0.02f;
    stateEstimationLayer.cornerEntryPredicted = true;
    stateEstimationLayer.stabilityMarginPct = 0.5f;
    stateEstimationLayer.lateralFeedbackLock = 0.0f;
    stateEstimationLayer.anticipatedWheelTorqueNm = 0.0f;
    rawCanInput.escOff = true;

    float result = calculatePredictiveLocks(30.0f);
    EXPECT_NEAR(result, 12.5f, 0.01f);
}

// oversteer + escOff=false → boost must be active (ESC on — normal mode)
TEST_F(LockCalculationTest, CornerEntryBoostActiveWhenOversteerEscOn) {
    stateEstimationLayer.chassisSlipDeviationRadS = -0.12f;
    stateEstimationLayer.cornerEntryPredicted = true;
    stateEstimationLayer.stabilityMarginPct = 0.5f;
    stateEstimationLayer.lateralFeedbackLock = 0.0f;
    stateEstimationLayer.anticipatedWheelTorqueNm = 0.0f;
    rawCanInput.escOff = false;

    float result = calculatePredictiveLocks(30.0f);
    // chassisBalanceAdjustment is negative on oversteer+escOn — the boost partially compensates
    // the point: the result is > 0 (boost is added) and ≠ 0
    EXPECT_GT(result, 0.0f);
}

// chassisSlipDeviation < oversteerThreshold (-0.0698 rad/s) → negative correction (reduces lock)
TEST_F(LockCalculationTest, ChassisBalanceOversteerReleasesLock) {
    // deviation = -0.2 rad/s, threshold = -0.0698
    // raw_delta = 0.1302 rad/s → 7.46 deg/s → gain=2.0 → -14.92%, escOff=false
    const HaldexControlConfig& cfg = activeConfig();
    float adjustment = calculateLateralChassisBalanceAdjustment(
        -0.2f, false, cfg, 0.0f);
    EXPECT_LT(adjustment, 0.0f);
    EXPECT_GE(adjustment, -cfg.balanceOversteerMaxRelease);
}
