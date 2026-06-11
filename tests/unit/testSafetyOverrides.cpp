#include "core/haldexController.hpp"

#include <gtest/gtest.h>

class SafetyOverridesTest : public ::testing::Test {
  protected:
    void SetUp() override {
        setupHaldexControl({HaldexControlConfig{}});
        setupVehiclePhysics(VehiclePhysicsConfig{});
        filterState = FilterState{};
        finalHaldexOutput = HaldexCommandOutput{};
        processedSignalsLayer = SignalProcessingLayer{};
        rawCanInput = CanInputLayer{};
    }

    static constexpr float dt = 0.010f;
};

// Ramp-up ≤ maxLockSlewRateUp (666.67 %/s) × dt = 6.667% per cycle
TEST_F(SafetyOverridesTest, SlewRateUpLimitedToMaxRate) {
    finalHaldexOutput.targetLockPct = 0.0f;
    applySlewRateFilter(100.0f, dt);
    // desiredChange = 100, maxUp = 666.67 * 0.010 = 6.667
    EXPECT_NEAR(finalHaldexOutput.targetLockPct, 6.667f, 0.01f);
}

// Ramp-down ≤ maxLockSlewRateDown (100 %/s) × dt = 1.0% per cycle
TEST_F(SafetyOverridesTest, SlewRateDownLimitedToMaxRate) {
    finalHaldexOutput.targetLockPct = 100.0f;
    applySlewRateFilter(0.0f, dt);
    // desiredChange = -100, maxDown = 100 * 0.010 = 1.0
    EXPECT_NEAR(finalHaldexOutput.targetLockPct, 99.0f, 0.01f);
}

// absActive = false → pass-through: the function returns currentTargetLock unchanged
TEST_F(SafetyOverridesTest, AbsOverridePassThroughWhenAbsInactive) {
    rawCanInput.absActive = false;
    float result = applyAbsSafetyOverride(50.0f, 15.0f, dt);
    EXPECT_FLOAT_EQ(result, 50.0f);
}

// absActive = true → absFadeLock decays exponentially from the initial value toward the floor
TEST_F(SafetyOverridesTest, AbsOverrideFadesLockExponentially) {
    finalHaldexOutput.targetLockPct = 60.0f;
    rawCanInput.absActive = true;

    // First cycle: initializes absFadeLock = 60, computes fade toward floor ≈ 8.4
    // alpha = 0.010/(0.060+0.010) = 0.143 → result ≈ 52.6 < 60
    float r1 = applyAbsSafetyOverride(60.0f, 20.0f, dt);
    EXPECT_LT(r1, 60.0f);

    // The second cycle continues the fade → r2 < r1
    float r2 = applyAbsSafetyOverride(r1, 20.0f, dt);
    EXPECT_LT(r2, r1);
}

// On a rapidly pulsing ABS (off phase < holdoff), the snapshot is not refreshed on reactivation
TEST_F(SafetyOverridesTest, AbsBrakeEventHoldsSnapshotOnRapidReactivation) {
    finalHaldexOutput.targetLockPct = 60.0f;
    rawCanInput.absActive = true;
    applyAbsSafetyOverride(60.0f, 20.0f, dt); // event start, snapshot = 60%

    rawCanInput.absActive = false;
    finalHaldexOutput.targetLockPct = 80.0f; // simulates recovery between pulses
    applyAbsSafetyOverride(80.0f, 20.0f, dt); // timer = holdoff - dt, does not expire

    rawCanInput.absActive = true;
    EXPECT_FLOAT_EQ(filterState.absInitialLock, 60.0f); // snapshot did not rise to 80
    float r = applyAbsSafetyOverride(80.0f, 20.0f, dt);
    EXPECT_LT(r, 60.0f); // fade continues downward from pulse 1
}

// kickdown = true → the function returns true regardless of speed
TEST_F(SafetyOverridesTest, KickdownAlwaysTriggersFullLock) {
    rawCanInput.kickdown = true;
    bool result = evaluateLaunchAndKickdown(50.0f, dt);
    EXPECT_TRUE(result);
}

// Regression: kickdown must not bypass the ABS holdoff during an inter-pulse gap.
// While the holdoff keeps absBrakeEventActive = true, a momentary absActive = false
// (gap between ABS pulses) must NOT let kickdown force 100% and slew-ratchet the output.
TEST_F(SafetyOverridesTest, KickdownDoesNotBypassAbsHoldoffDuringPulseGap) {
    stateEstimationLayer = StateEstimationLayer{};
    rawCanInput.gear = 2;
    rawCanInput.vehicleSpeedKmh = 50.0f;
    rawCanInput.brakePressure = 60.0f; // hard braking → normal lock fully attenuated
    rawCanInput.kickdown = true;
    // Equal wheel speeds → no reactive slip contribution
    rawCanInput.wheelSpeedFl = rawCanInput.wheelSpeedFr = 50.0f;
    rawCanInput.wheelSpeedRl = rawCanInput.wheelSpeedRr = 50.0f;

    // Cycle 1: ABS pulse ON → establishes the brake event (absBrakeEventActive = true)
    rawCanInput.absActive = true;
    haldexControllerExecutionTask(dt);

    // Cycles 2..6: inter-pulse gap (ABS momentarily off) with kickdown held.
    // 5 * 10ms = 50ms < 300ms holdoff → absBrakeEventActive stays true → kickdown stays gated.
    rawCanInput.absActive = false;
    for (int i = 0; i < 5; i++) {
        haldexControllerExecutionTask(dt);
    }

    EXPECT_TRUE(filterState.absBrakeEventActive); // still inside the holdoff window
    EXPECT_LT(finalHaldexOutput.targetLockPct, 1.0f); // no ratchet toward 100%
}

// launchFlagHoldoffS = 0.10s: over 5 cycles (50ms) of packet loss launch stays active
TEST_F(SafetyOverridesTest, LaunchFlagHoldoffProtectsAgainstPacketLoss) {
    processedSignalsLayer.normalizedThrottle = 0.90f;
    processedSignalsLayer.steeringAngleRad = 0.0f;
    rawCanInput.launchControlFlag = true;

    // Activates launch → launchControlHoldTimer = 1.5s, holdoffTimer = 0.10s
    evaluateLaunchAndKickdown(0.5f, dt);
    EXPECT_TRUE(stateEstimationLayer.launchActive);

    // Simulates 5 cycles of packet loss (50ms < 100ms holdoff)
    rawCanInput.launchControlFlag = false;
    for (int i = 0; i < 5; i++) {
        evaluateLaunchAndKickdown(0.5f, dt);
    }

    // holdoffTimer = 0.10 - 5*0.010 = 0.05 > 0 → debouncedLaunchFlag = true
    // launch conditions still hold → holdTimer reset → launchActive = true
    EXPECT_TRUE(stateEstimationLayer.launchActive);
}

// steering > launchSteeringMaxColumnRad (0.52f) → launch does not activate
TEST_F(SafetyOverridesTest, LaunchBlockedWhenSteeringExceedsThreshold) {
    processedSignalsLayer.normalizedThrottle = 0.90f;
    rawCanInput.launchControlFlag = true;

    // Above the threshold — launch must be blocked
    processedSignalsLayer.steeringAngleRad = 0.55f;
    evaluateLaunchAndKickdown(0.5f, dt);
    EXPECT_FALSE(stateEstimationLayer.launchActive);

    // Just below the threshold — launch must activate
    filterState = FilterState{};
    processedSignalsLayer.steeringAngleRad = 0.51f;
    evaluateLaunchAndKickdown(0.5f, dt);
    EXPECT_TRUE(stateEstimationLayer.launchActive);
}

// brakePressure < brakeAttenuationStartBar (15 bar) → attenuation = 0.0
TEST_F(SafetyOverridesTest, BrakeAttenuationZeroBelowStartPressure) {
    rawCanInput.brakePressure = 10.0f;
    EXPECT_FLOAT_EQ(calculateOutputAttenuation(rawCanInput), 0.0f);

    // Exactly at the threshold → (15-15)/45 = 0
    rawCanInput.brakePressure = 15.0f;
    EXPECT_FLOAT_EQ(calculateOutputAttenuation(rawCanInput), 0.0f);

    // Above the threshold → (60-15)/45 = 1.0
    rawCanInput.brakePressure = 60.0f;
    EXPECT_FLOAT_EQ(calculateOutputAttenuation(rawCanInput), 1.0f);
}

// Reverse, no slip, no lateral g → lock must be 0% (no feedforward, no base speed)
TEST_F(SafetyOverridesTest, ZeroLockInReverseNoSlipNoLateral) {
    stateEstimationLayer = StateEstimationLayer{};
    rawCanInput.reverseGear = true;
    rawCanInput.gear = 0;
    rawCanInput.throttlePct = 80.0f;
    rawCanInput.vehicleSpeedKmh = 5.0f;
    rawCanInput.actualTorqueSumNm = 200.0f;
    rawCanInput.lateralAccelG = 0.0f;
    // All wheels at equal speed = the vehicle speed → no slip
    rawCanInput.wheelSpeedFl = rawCanInput.wheelSpeedFr = 5.0f;
    rawCanInput.wheelSpeedRl = rawCanInput.wheelSpeedRr = 5.0f;

    haldexControllerExecutionTask(dt);
    EXPECT_FLOAT_EQ(finalHaldexOutput.targetLockPct, 0.0f);
}

// Reverse + front tires slipping (front >> rear) → reactive lock > 0
TEST_F(SafetyOverridesTest, ReactiveSlipActiveInReverse) {
    stateEstimationLayer = StateEstimationLayer{};
    rawCanInput.reverseGear = true;
    rawCanInput.gear = 0;
    rawCanInput.throttlePct = 60.0f;
    rawCanInput.vehicleSpeedKmh = 8.0f;
    rawCanInput.actualTorqueSumNm = 200.0f;
    rawCanInput.lateralAccelG = 0.0f;
    // Front spins faster than rear → frontRearSlip > 0.05 m/s (threshold)
    rawCanInput.wheelSpeedFl = rawCanInput.wheelSpeedFr = 12.0f; // km/h
    rawCanInput.wheelSpeedRl = rawCanInput.wheelSpeedRr = 8.0f;

    haldexControllerExecutionTask(dt);
    EXPECT_GT(finalHaldexOutput.targetLockPct, 0.0f);
}

// Reverse + lateral g (0.8g) without slip → lateralFeedbackLock active (up to 30%)
TEST_F(SafetyOverridesTest, LateralFeedbackActiveInReverse) {
    stateEstimationLayer = StateEstimationLayer{};
    rawCanInput.reverseGear = true;
    rawCanInput.gear = 0;
    rawCanInput.vehicleSpeedKmh = 10.0f;
    rawCanInput.lateralAccelG = 0.8f; // → lateralFeedbackLock = 30%
    // No slip
    rawCanInput.wheelSpeedFl = rawCanInput.wheelSpeedFr = 10.0f;
    rawCanInput.wheelSpeedRl = rawCanInput.wheelSpeedRr = 10.0f;

    haldexControllerExecutionTask(dt);
    EXPECT_GT(finalHaldexOutput.targetLockPct, 0.0f);
}

// Forward mode: reactiveSlipLock does not exceed maxReactiveSlipLockCap (50%)
TEST_F(SafetyOverridesTest, ForwardReactiveCapUnchangedAt50Pct) {
    stateEstimationLayer = StateEstimationLayer{};
    rawCanInput.reverseGear = false;
    rawCanInput.gear = 2;
    rawCanInput.vehicleSpeedKmh = 30.0f;
    // Huge slip: front 50 km/h, rear 30 km/h
    rawCanInput.wheelSpeedFl = rawCanInput.wheelSpeedFr = 50.0f;
    rawCanInput.wheelSpeedRl = rawCanInput.wheelSpeedRr = 30.0f;

    haldexControllerExecutionTask(dt);
    EXPECT_LE(stateEstimationLayer.reactiveSlipLock, 50.0f);
}
