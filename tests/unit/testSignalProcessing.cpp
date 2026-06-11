#include "core/haldexController.hpp"

#include <gtest/gtest.h>

class SignalProcessingTest : public ::testing::Test {
  protected:
    void SetUp() override {
        setupHaldexControl({HaldexControlConfig{}});
        setupVehiclePhysics(VehiclePhysicsConfig{});
        processedSignalsLayer = SignalProcessingLayer{};
        filterState = FilterState{};
        rawCanInput = CanInputLayer{};
    }

    static constexpr float dt = 0.010f;
};

// Delta > 8 m/s (> 28.8 km/h between two cycles) → wheelValid = false for this cycle
TEST_F(SignalProcessingTest, WheelSpikeInvalidForOneCycle) {
    rawCanInput.vehicleSpeedKmh = 40.0f;
    rawCanInput.wheelSpeedFl = rawCanInput.wheelSpeedFr =
        rawCanInput.wheelSpeedRl = rawCanInput.wheelSpeedRr = 40.0f;
    // Cycle 1: prevRaw = 0 → delta = 11.11 m/s (spike, all invalid)
    // Cycle 2: prevRaw = 11.11 → delta = 0 (all valid)
    processSignalsFiltered(rawCanInput, processedSignalsLayer, filterState, dt);
    processSignalsFiltered(rawCanInput, processedSignalsLayer, filterState, dt);
    ASSERT_TRUE(processedSignalsLayer.wheelValid[0]);

    // Spike FL by 30 km/h → delta = 8.33 m/s > 8.0 → FL invalid
    rawCanInput.wheelSpeedFl = 70.0f;
    processSignalsFiltered(rawCanInput, processedSignalsLayer, filterState, dt);
    EXPECT_FALSE(processedSignalsLayer.wheelValid[0]);
    EXPECT_TRUE(processedSignalsLayer.wheelValid[1]);
    EXPECT_TRUE(processedSignalsLayer.wheelValid[2]);
    EXPECT_TRUE(processedSignalsLayer.wheelValid[3]);
}

// frozenCounter > 20 (21 cycles × 10ms = 210ms) → wheelValid = false
TEST_F(SignalProcessingTest, FrozenWheelInvalidAfter200ms) {
    rawCanInput.vehicleSpeedKmh = 40.0f;
    rawCanInput.wheelSpeedFl = rawCanInput.wheelSpeedFr =
        rawCanInput.wheelSpeedRl = rawCanInput.wheelSpeedRr = 40.0f;
    // One warmup cycle → establishes prevRaw[0] = 40 * 0.2778 m/s
    processSignalsFiltered(rawCanInput, processedSignalsLayer, filterState, dt);

    // FL: delta = 0 every cycle → frozenCounter increments
    // FR/RL/RR: ±0.15 km/h noise → delta ≈ 0.083 m/s > 0.028 → counter reset
    for (int i = 0; i < 21; i++) {
        float noise = (i % 2 == 0) ? 0.15f : -0.15f;
        rawCanInput.wheelSpeedFl = 40.0f;
        rawCanInput.wheelSpeedFr = 40.0f + noise;
        rawCanInput.wheelSpeedRl = 40.0f + noise;
        rawCanInput.wheelSpeedRr = 40.0f + noise;
        processSignalsFiltered(rawCanInput, processedSignalsLayer, filterState, dt);
    }
    EXPECT_FALSE(processedSignalsLayer.wheelValid[0]);
    EXPECT_TRUE(processedSignalsLayer.wheelValid[1]);
}

// A wheel deviating > 5 m/s from the median of all 4 → inter-wheel exclusion
TEST_F(SignalProcessingTest, InterWheelPlausibilityExcludes5mpsOutlier) {
    rawCanInput.vehicleSpeedKmh = 40.0f;
    rawCanInput.wheelSpeedFl = rawCanInput.wheelSpeedFr =
        rawCanInput.wheelSpeedRl = 40.0f;
    // RR = 60 km/h = 16.67 m/s; median = 11.11 m/s; deviation = 5.56 > 5.0 → invalid
    rawCanInput.wheelSpeedRr = 60.0f;
    // Cycle 1: prevRaw = 0 → all spike (invalid). Needed to establish the baseline.
    processSignalsFiltered(rawCanInput, processedSignalsLayer, filterState, dt);
    // Cycle 2: prevRaw = raw (delta = 0). Spike check OK. Inter-wheel excludes RR.
    processSignalsFiltered(rawCanInput, processedSignalsLayer, filterState, dt);
    EXPECT_FALSE(processedSignalsLayer.wheelValid[3]);
    EXPECT_TRUE(processedSignalsLayer.wheelValid[0]);
    EXPECT_TRUE(processedSignalsLayer.wheelValid[1]);
    EXPECT_TRUE(processedSignalsLayer.wheelValid[2]);
}

// Low-pass effect: the steering rate filter output < raw rate (step input)
TEST_F(SignalProcessingTest, SteeringRateFilterSmoothsOutput) {
    // Raw rate = 90 deg (1.5708 rad) / 0.010 s = 157.08 rad/s
    // alpha = dt / (tau + dt) = 0.010 / 0.030 = 0.333 → filtered = 52.4 rad/s < 157.08
    float angleRad = 90.0f * 0.0174532925f;
    float filteredRate = calculateFilteredSteeringRate(angleRad, filterState, dt);
    EXPECT_LT(std::abs(filteredRate), 157.08f);
    EXPECT_GT(std::abs(filteredRate), 0.0f);
}

// throttlePct / 100 → normalizedThrottle, clamp [0.0, 1.0]
TEST_F(SignalProcessingTest, ThrottleNormalizationClampsTo0_1) {
    rawCanInput.vehicleSpeedKmh = 40.0f;
    rawCanInput.wheelSpeedFl = rawCanInput.wheelSpeedFr =
        rawCanInput.wheelSpeedRl = rawCanInput.wheelSpeedRr = 40.0f;

    rawCanInput.throttlePct = 50.0f;
    processSignalsFiltered(rawCanInput, processedSignalsLayer, filterState, dt);
    EXPECT_NEAR(processedSignalsLayer.normalizedThrottle, 0.5f, 0.001f);

    rawCanInput.throttlePct = 150.0f;
    processSignalsFiltered(rawCanInput, processedSignalsLayer, filterState, dt);
    EXPECT_FLOAT_EQ(processedSignalsLayer.normalizedThrottle, 1.0f);

    rawCanInput.throttlePct = -10.0f;
    processSignalsFiltered(rawCanInput, processedSignalsLayer, filterState, dt);
    EXPECT_FLOAT_EQ(processedSignalsLayer.normalizedThrottle, 0.0f);
}
