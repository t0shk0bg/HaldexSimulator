#include "core/haldexController.hpp"

#include <cstdio>
#include <gtest/gtest.h>

static void resetControllerState() {
    processedSignalsLayer = SignalProcessingLayer{};
    stateEstimationLayer = StateEstimationLayer{};
    filterState = FilterState{};
    finalHaldexOutput = HaldexCommandOutput{};
}

class FrozenSensorTest : public ::testing::Test {
  protected:
    void SetUp() override {
        setupHaldexControl({HaldexControlConfig{}});
        setupVehiclePhysics(VehiclePhysicsConfig{});
        resetControllerState();

        rawCanInput = CanInputLayer{};
        rawCanInput.vehicleSpeedKmh = 40.0f;
        rawCanInput.wheelSpeedFl = 40.0f;
        rawCanInput.wheelSpeedFr = 40.0f;
        rawCanInput.wheelSpeedRl = 40.0f;
        rawCanInput.wheelSpeedRr = 40.0f;
        rawCanInput.throttlePct = 30.0f;
        rawCanInput.gear = 2;
    }

    static constexpr float dt = 0.010f;

    void runWarmup(int cycles = 50) {
        for (int i = 0; i < cycles; i++) {
            haldexControllerExecutionTask(dt);
        }
    }

    void freezeRrAndRun(int cycles) {
        for (int i = 0; i < cycles; i++) {
            rawCanInput.wheelSpeedRr = 0.0f;
            haldexControllerExecutionTask(dt);
        }
    }

    // Warmup and freeze with realistic sensor noise (±0.15 km/h) on the valid wheels.
    // Prevents the frozen-counter from triggering on FL/FR/RL at constant speed.
    void runNoisyWarmup(int cycles = 50) {
        for (int i = 0; i < cycles; i++) {
            float noise = (i % 2 == 0) ? 0.15f : -0.15f;
            rawCanInput.wheelSpeedFl = 40.0f + noise;
            rawCanInput.wheelSpeedFr = 40.0f + noise;
            rawCanInput.wheelSpeedRl = 40.0f + noise;
            rawCanInput.wheelSpeedRr = 40.0f + noise;
            haldexControllerExecutionTask(dt);
        }
    }

    void freezeRrAndRunNoisy(int cycles) {
        for (int i = 0; i < cycles; i++) {
            float noise = (i % 2 == 0) ? 0.15f : -0.15f;
            rawCanInput.wheelSpeedFl = 40.0f + noise;
            rawCanInput.wheelSpeedFr = 40.0f + noise;
            rawCanInput.wheelSpeedRl = 40.0f + noise;
            rawCanInput.wheelSpeedRr = 0.0f;
            haldexControllerExecutionTask(dt);
        }
    }
};

TEST_F(FrozenSensorTest, RRWheelFlaggedAfter21Cycles) {
    runWarmup();
    freezeRrAndRun(21); // 210ms — 1 cycle above the threshold of 20
    EXPECT_FALSE(processedSignalsLayer.wheelValid[3]);
}

TEST_F(FrozenSensorTest, ConfidenceDropsBelow58PctOnOneFrozenWheel) {
    runWarmup();
    freezeRrAndRun(25);
    EXPECT_LE(stateEstimationLayer.wheelDataConfidence, 0.58f);
}

TEST_F(FrozenSensorTest, FrontWheelsRemainsValidDuringRRFreeze) {
    // We use the noisy version — ±0.15 km/h noise (delta ~0.083 m/s > the 0.028 m/s threshold)
    // resets the frozenCounter of the valid wheels every cycle, like a real ABS sensor.
    runNoisyWarmup();
    freezeRrAndRunNoisy(50);
    EXPECT_TRUE(processedSignalsLayer.wheelValid[0]);
    EXPECT_TRUE(processedSignalsLayer.wheelValid[1]);
}

TEST_F(FrozenSensorTest, SlipResetsToZeroWhenRRExcludedFromCalc) {
    runWarmup();
    freezeRrAndRun(30);
    // With RR frozen: the rear averaging is based only on RL (valid)
    // RL == 40 km/h == ~11.11 m/s, FL/FR ~ 11.11 m/s — slip ≈ 0
    EXPECT_NEAR(stateEstimationLayer.frontRearSlipMps, 0.0f, 0.5f);
}
