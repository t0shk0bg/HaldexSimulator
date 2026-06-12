#pragma once
// ============================================================================
// SYSTEM REVISION: V1.1.0
// ============================================================================

#include "types/canTypes.hpp"
#include "types/configTypes.hpp"
#include "types/controllerState.hpp"
#include "types/physicsTypes.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>

// ----------------------------------------------------------------------------
// Controller global state
// ----------------------------------------------------------------------------
extern HaldexCalibration calib;
extern DriveMode driveMode;
extern VehiclePhysicsConfig physConfig;
extern CanInputLayer rawCanInput;
extern SignalProcessingLayer processedSignalsLayer;
extern StateEstimationLayer stateEstimationLayer;
extern FilterState filterState;
extern HaldexCommandOutput finalHaldexOutput;

// ----------------------------------------------------------------------------
// Inline helpers
// ----------------------------------------------------------------------------
inline const HaldexControlConfig& activeConfig() {
    return calib.configs[static_cast<int>(driveMode)];
}

inline float clamp(float value, float minVal, float maxVal) {
    return (value < minVal) ? minVal : ((value > maxVal) ? maxVal : value);
}

// ----------------------------------------------------------------------------
// Configuration and initialization
// ----------------------------------------------------------------------------
void setupHaldexControl(std::initializer_list<HaldexControlConfig> configsList);
void setupVehiclePhysics(const VehiclePhysicsConfig& newPhysCfg);
void updateDriveModeFromCan(const CanInputLayer& input);

// ----------------------------------------------------------------------------
// Signal processing
// ----------------------------------------------------------------------------
void processSignalsFiltered(const CanInputLayer& input, SignalProcessingLayer& processed, FilterState& fState, float dt);
float calculateFilteredSteeringRate(float currentAngleRad, FilterState& fState, float dt);
float calculateFilteredYawAccel(float currentYawRateRadS, FilterState& fState, float dt);

// ----------------------------------------------------------------------------
// Dynamic state estimation
// ----------------------------------------------------------------------------
void estimateSteeringAndYaw(const SignalProcessingLayer& processed, FilterState& fState, StateEstimationLayer& state, float V, float dt);
void estimateWheelTorque(const CanInputLayer& input, const VehiclePhysicsConfig& physCfg, StateEstimationLayer& state);
void estimateReactiveSlip(const CanInputLayer& input, const SignalProcessingLayer& processed, StateEstimationLayer& state, float vehicleSpeedMps);
void estimateLateralDynamics(const CanInputLayer& input, const VehiclePhysicsConfig& physCfg, const SignalProcessingLayer& processed, StateEstimationLayer& state, float vehicleSpeedMps);

// ----------------------------------------------------------------------------
// Lock calculation
// ----------------------------------------------------------------------------
inline bool isOversteerActive(float slipDeviationRadS) {
    return slipDeviationRadS < activeConfig().balanceOversteerThresholdRadS;
}

float calculateBaseSpeedLock(float speedMps);
float calculateFeedforwardTorqueLock(float wheelTorqueNm);
float calculateLateralChassisBalanceAdjustment(float slipDeviationRadS, bool escOff, const HaldexControlConfig& cfg, float rearOverrunSlipMps);
float calculateParkingDegradation(float V, float steeringAngleRad);
float calculateOutputAttenuation(const CanInputLayer& input);
float calculatePredictiveLocks(float V);

// ----------------------------------------------------------------------------
// Safety limits and output filter
// ----------------------------------------------------------------------------
float applyAbsSafetyOverride(float currentTargetLock, float V, float safeDt);
void applySlewRateFilter(float rawTargetLock, float safeDt);
bool evaluateLaunchAndKickdown(float V, float safeDt);

// ----------------------------------------------------------------------------
// Main loop
// ----------------------------------------------------------------------------
float prepareCycleMetrics(float dtSeconds, float& safeDt, float& normalAttenuation);
void haldexControllerExecutionTask(float dtSeconds);
