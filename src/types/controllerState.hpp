#pragma once

struct SignalProcessingLayer {
    float vehicleSpeedMps = 0.0f;
    float wheelSpeedsMps[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float normalizedThrottle = 0.0f;
    float steeringAngleRad = 0.0f;
    float steeringRateRadS = 0.0f;
    float yawRateRadS = 0.0f;
    float wheelMicroOscillationsMps = 0.0f;
    bool wheelValid[4] = {true, true, true, true};
};

struct FilterState {
    float previousSteeringAngleRad = 0.0f;
    float filteredSteeringRateRadS = 0.0f;
    float laggedExpectedYawRadS = 0.0f;
    float previousYawRateRadS = 0.0f;
    float launchControlHoldTimer = 0.0f;
    float launchFlagHoldoffTimer = 0.0f;

    bool absBrakeEventActive = false;
    float absBrakeEventEndTimer = 0.0f;

    bool absFadeInitialized = false;
    float absFadeLock = 0.0f;
    float absInitialLock = 0.0f;

    float prevRawWheels[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    int frozenCounters[4] = {0, 0, 0, 0};
};

struct StateEstimationLayer {
    float frontRearSlipMps = 0.0f;
    float rearOverrunSlipMps = 0.0f;
    float anticipatedWheelTorqueNm = 0.0f;
    bool cornerEntryPredicted = false;
    bool launchActive = false;
    float chassisSlipDeviationRadS = 0.0f;
    float tireGripAvailablePct = 1.0f;
    float lateralFeedbackLock = 0.0f;
    float stabilityMarginPct = 1.0f;
    float wheelDataConfidence = 1.0f;
    float dynamicSlipGain = 0.0f;
    float steeringRateRadS = 0.0f;

    // Telemetry cache
    float baseSpeedLock = 0.0f;
    float proactiveTorqueLock = 0.0f;
    float reactiveSlipLock = 0.0f;
    float chassisBalanceAdjustment = 0.0f;
};

struct HaldexCommandOutput {
    float targetLockPct = 0.0f;
};
