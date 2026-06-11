#include "haldexController.hpp"

// ----------------------------------------------------------------------------
// Global state
// ----------------------------------------------------------------------------
HaldexCalibration calib;
DriveMode driveMode = DriveMode::NORMAL;
VehiclePhysicsConfig physConfig;
CanInputLayer rawCanInput;
SignalProcessingLayer processedSignalsLayer;
StateEstimationLayer stateEstimationLayer;
FilterState filterState;
HaldexCommandOutput finalHaldexOutput;

// ----------------------------------------------------------------------------
// Internal helpers (file-local)
// ----------------------------------------------------------------------------
static float calculateChassisSlipDeviation(float laggedExpectedYawRadS, float realYawRadS, float vehicleSpeedMps, float steeringAngleRad) {
    if (std::abs(steeringAngleRad) <= 0.02f || vehicleSpeedMps <= 5.5f) {
        return 0.0f;
    }

    return std::abs(laggedExpectedYawRadS) - std::abs(realYawRadS);
}

static float calculateKinematicExpectedYaw(float currentSteerAngleRad, float V, float longitudinalG) {
    float steeringAtTiresRad = currentSteerAngleRad / activeConfig().steeringRatio;
    float expectedYawRadS = (steeringAtTiresRad * V) / physConfig.wheelbase;

    float yawGain = 1.0f + (V * activeConfig().yawGainLinearCoeff) - (V * V * activeConfig().yawGainQuadraticCoeff);
    float loadTransfer = 1.0f - (longitudinalG * 0.3f);

    return expectedYawRadS * std::max(0.1f, yawGain) * std::max(0.1f, loadTransfer);
}

static bool validateChassisResponse(float filteredSteerRateRadS, float expectedYawRadS, float realYawRadS, FilterState& fState, float V, float dt) {
    if (V < 4.16f) {
        return true;
    }

    float tau = clamp(activeConfig().chassisLagMaxS - (V * activeConfig().chassisLagScale), activeConfig().chassisLagMinS, activeConfig().chassisLagMaxS);
    float alphaLag = dt / (tau + dt);
    fState.laggedExpectedYawRadS += alphaLag * (expectedYawRadS - fState.laggedExpectedYawRadS);

    float realYawRateAbs = std::abs(realYawRadS);
    float laggedExpectedAbs = std::abs(fState.laggedExpectedYawRadS);

    bool signMatch = (filteredSteerRateRadS * realYawRadS) > 0.0f;
    bool dynamicSlalomTransition = std::abs(filteredSteerRateRadS) > activeConfig().slalomTransitionThresholdRadS;

    bool directionOk = signMatch || dynamicSlalomTransition;
    bool lowerBound = realYawRateAbs > (laggedExpectedAbs * 0.40f);
    bool upperBound = realYawRateAbs < (laggedExpectedAbs * 1.35f);

    bool isOversteering = isOversteerActive(stateEstimationLayer.chassisSlipDeviationRadS);

    if (isOversteering && rawCanInput.escOff) {
        return directionOk;
    }

    if (dynamicSlalomTransition) {
        return directionOk && upperBound;
    }

    return directionOk && lowerBound && upperBound;
}

// ----------------------------------------------------------------------------
// Configuration and initialization
// ----------------------------------------------------------------------------
void setupHaldexControl(std::initializer_list<HaldexControlConfig> configsList) {
    int index = 0;
    const int maxCount = static_cast<int>(DriveMode::COUNT);

    for (const auto& cfg : configsList) {
        if (index >= maxCount) {
            break;
        }

        calib.configs[index] = cfg;
        index++;
    }
}

void setupVehiclePhysics(const VehiclePhysicsConfig& newPhysCfg) {
    physConfig = newPhysCfg;
    float fd1Max = physConfig.engineMaxTorque * physConfig.gearRatios[1] * physConfig.finalDrive1 * physConfig.drivetrainEfficiency;
    float fd2Max = physConfig.engineMaxTorque * physConfig.gearRatios[5] * physConfig.finalDrive2 * physConfig.drivetrainEfficiency;
    physConfig.maxWheelTorque = std::max(fd1Max, fd2Max);
}

void updateDriveModeFromCan(const CanInputLayer& input) {
    driveMode = (input.selectedDriveMode == 1) ? DriveMode::SPORT : DriveMode::NORMAL;
}

// ----------------------------------------------------------------------------
// Signal processing
// ----------------------------------------------------------------------------
void processSignalsFiltered(const CanInputLayer& input, SignalProcessingLayer& processed, FilterState& fState, float dt) {
    const float kmhToMps = 0.277777778f;
    const float degToRad = 0.0174532925f;

    float alphaV = dt / (activeConfig().signalFilterTimeConstant + dt);

    processed.vehicleSpeedMps += alphaV * ((input.vehicleSpeedKmh * kmhToMps) - processed.vehicleSpeedMps);

    float rawWheels[4] = {
        input.wheelSpeedFl * kmhToMps, input.wheelSpeedFr * kmhToMps,
        input.wheelSpeedRl * kmhToMps, input.wheelSpeedRr * kmhToMps};

    for (int i = 0; i < 4; i++) {
        processed.wheelValid[i] = true;
    }

    for (int i = 0; i < 4; i++) {
        float delta = std::abs(rawWheels[i] - fState.prevRawWheels[i]);

        if (delta > 8.0f) {
            processed.wheelValid[i] = false;
            fState.frozenCounters[i] = 0;
        } else if (processed.vehicleSpeedMps > 0.55f && delta < 0.028f) {
            if (++fState.frozenCounters[i] > 20) {
                processed.wheelValid[i] = false;
            }
        } else {
            fState.frozenCounters[i] = 0;
        }

        fState.prevRawWheels[i] = rawWheels[i];
    }

    // Inter-wheel plausibility: if one wheel deviates >5 m/s from the median
    {
        float s[4] = {rawWheels[0], rawWheels[1], rawWheels[2], rawWheels[3]};

        for (int a = 0; a < 3; a++) {
            for (int b = a + 1; b < 4; b++) {
                if (s[b] < s[a]) {
                    float t = s[a];
                    s[a] = s[b];
                    s[b] = t;
                }
            }
        }

        float median = (s[1] + s[2]) * 0.5f;

        for (int i = 0; i < 4; i++) {
            if (std::abs(rawWheels[i] - median) > 5.0f) {
                processed.wheelValid[i] = false;
            }
        }
    }

    float totalOscillationDeviation = 0.0f;

    for (int i = 0; i < 4; i++) {
        processed.wheelSpeedsMps[i] += alphaV * (rawWheels[i] - processed.wheelSpeedsMps[i]);
        totalOscillationDeviation += std::abs(rawWheels[i] - processed.wheelSpeedsMps[i]);
    }

    float alphaOsc = dt / (activeConfig().wheelOscillationFilterTime + dt);
    processed.wheelMicroOscillationsMps += alphaOsc * ((totalOscillationDeviation * 0.25f) - processed.wheelMicroOscillationsMps);
    processed.normalizedThrottle = clamp(input.throttlePct * 0.01f, 0.0f, 1.0f);

    float steerDeg = activeConfig().steeringInputIsColumnAngle ? input.steeringAngleDeg : input.steeringAngleDeg * activeConfig().steeringRatio;
    processed.steeringAngleRad += alphaV * ((steerDeg * degToRad) - processed.steeringAngleRad);
    processed.yawRateRadS = input.yawRateDegS * degToRad;
}

float calculateFilteredSteeringRate(float currentAngleRad, FilterState& fState, float dt) {
    float rawRateRadS = (currentAngleRad - fState.previousSteeringAngleRad) / dt;
    fState.previousSteeringAngleRad = currentAngleRad;

    float alpha = dt / (activeConfig().steeringRateFilterTime + dt);
    fState.filteredSteeringRateRadS += alpha * (rawRateRadS - fState.filteredSteeringRateRadS);

    return fState.filteredSteeringRateRadS;
}

// ----------------------------------------------------------------------------
// Dynamic state estimation
// ----------------------------------------------------------------------------
void estimateSteeringAndYaw(const SignalProcessingLayer& processed, FilterState& fState, StateEstimationLayer& state, float V, float dt) {
    state.steeringRateRadS = calculateFilteredSteeringRate(processed.steeringAngleRad, fState, dt);
    fState.previousYawRateRadS = processed.yawRateRadS;

    float dynamicThresholdRadS = clamp(2.44f - (V * 0.043f), 1.04f, 2.44f);
    bool steeringTrigger = std::abs(state.steeringRateRadS) > dynamicThresholdRadS;

    float expectedYawRadS = calculateKinematicExpectedYaw(processed.steeringAngleRad, V, rawCanInput.longitudinalAccelG);
    bool chassisOk = validateChassisResponse(state.steeringRateRadS, expectedYawRadS, processed.yawRateRadS, fState, V, dt);

    state.chassisSlipDeviationRadS = calculateChassisSlipDeviation(fState.laggedExpectedYawRadS, processed.yawRateRadS, V, processed.steeringAngleRad);
    state.cornerEntryPredicted = steeringTrigger && chassisOk;
}

void estimateWheelTorque(const CanInputLayer& input, const VehiclePhysicsConfig& physCfg, StateEstimationLayer& state) {
    if (input.reverseGear || input.gear <= 0 || input.gear > 7) {
        state.anticipatedWheelTorqueNm = 0.0f;
        return;
    }

    float iGear = (input.gear >= 1 && input.gear <= 7) ? physCfg.gearRatios[input.gear] : physCfg.gearRatios[7];
    float iFinal = (input.gear <= 4) ? physCfg.finalDrive1 : physCfg.finalDrive2;
    state.anticipatedWheelTorqueNm = clamp(input.actualTorqueSumNm, 0.0f, physCfg.engineMaxTorque) * iGear * iFinal * physCfg.drivetrainEfficiency;
}

void estimateReactiveSlip(const CanInputLayer& input, const SignalProcessingLayer& processed, StateEstimationLayer& state, float vehicleSpeedMps) {
    float sumFront = 0.0f;
    int cntFront = 0;
    float sumRear = 0.0f;
    int cntRear = 0;

    if (processed.wheelValid[0]) {
        sumFront += processed.wheelSpeedsMps[0];
        cntFront++;
    }

    if (processed.wheelValid[1]) {
        sumFront += processed.wheelSpeedsMps[1];
        cntFront++;
    }

    if (processed.wheelValid[2]) {
        sumRear += processed.wheelSpeedsMps[2];
        cntRear++;
    }

    if (processed.wheelValid[3]) {
        sumRear += processed.wheelSpeedsMps[3];
        cntRear++;
    }

    if (vehicleSpeedMps < 0.55f || cntFront == 0 || cntRear == 0) {
        state.frontRearSlipMps = 0.0f;
        state.rearOverrunSlipMps = 0.0f;
    } else {
        float avgFront = sumFront / cntFront;
        float avgRear = sumRear / cntRear;
        state.frontRearSlipMps = std::max(0.0f, avgFront - avgRear);
        state.rearOverrunSlipMps = std::max(0.0f, avgRear - avgFront);
    }

    if (input.escOff) {
        state.wheelDataConfidence = 1.0f;
    } else {
        float oscillationConfidence = (processed.wheelMicroOscillationsMps > 0.45f) ? clamp(1.0f - ((processed.wheelMicroOscillationsMps - 0.45f) / 1.5f), 0.15f, 1.0f) : 1.0f;
        int invalidCount = 0;

        for (int i = 0; i < 4; i++) {
            if (!processed.wheelValid[i]) {
                invalidCount++;
            }
        }

        float sensorPenalty = 1.0f - (invalidCount * 0.425f);
        state.wheelDataConfidence = clamp(std::min(oscillationConfidence, sensorPenalty), 0.15f, 1.0f);
    }

    float loadTransferMultiplier = 1.0f + (input.longitudinalAccelG * 0.5f);
    state.dynamicSlipGain = activeConfig().slipReactiveGain * clamp(loadTransferMultiplier, activeConfig().slipGainMultiplierMin, activeConfig().slipGainMultiplierMax);
    state.frontRearSlipMps *= state.wheelDataConfidence;
    state.rearOverrunSlipMps *= state.wheelDataConfidence;
}

void estimateLateralDynamics(const CanInputLayer& input, const VehiclePhysicsConfig& physCfg, const SignalProcessingLayer& processed, StateEstimationLayer& state, float vehicleSpeedMps) {
    float latGAbs = std::abs(input.lateralAccelG);
    state.lateralFeedbackLock = clamp((latGAbs / 0.8f) * 30.0f, 0.0f, 30.0f);
    state.stabilityMarginPct = clamp(1.0f - (latGAbs / physCfg.tireMaxLateralG), 0.0f, 1.0f);

    if (input.escOff) {
        state.tireGripAvailablePct = 1.0f;
    } else {
        float longN = input.longitudinalAccelG / physCfg.tireMaxLongG;
        float latN = input.lateralAccelG / physCfg.tireMaxLateralG;
        float gripSquared = 1.0f - (longN * longN + latN * latN);
        state.tireGripAvailablePct = (gripSquared > 0.0f) ? std::sqrt(gripSquared) : 0.0f;
    }
}

// ----------------------------------------------------------------------------
// Lock calculation
// ----------------------------------------------------------------------------
float calculateOutputAttenuation(const CanInputLayer& input) {
    return clamp((input.brakePressure - activeConfig().brakeAttenuationStartBar) / activeConfig().brakeAttenuationRangeBar, 0.0f, 1.0f);
}

float calculateBaseSpeedLock(float speedMps) {
    if (speedMps >= activeConfig().baseLockHighSpeedCutMps) {
        return 0.0f;
    }

    float t = speedMps / activeConfig().baseLockHighSpeedCutMps;

    return activeConfig().baseLockLowSpeed * (1.0f - std::pow(t, activeConfig().baseLockSpeedExponent));
}

float calculateFeedforwardTorqueLock(float wheelTorqueNm) {
    float normalized = wheelTorqueNm / std::max(10.0f, physConfig.maxWheelTorque);
    float base = clamp(normalized * activeConfig().throttleProactiveGain, 0.0f, activeConfig().throttleProactiveGain);
    float req = rawCanInput.filteredRequestedTorqueNm;
    float act = rawCanInput.actualTorqueSumNm;

    if (req < 20.0f) {
        return base;
    }

    return base * clamp(act / req, 0.0f, 1.0f);
}

float calculateLateralChassisBalanceAdjustment(float slipDeviationRadS, bool escOff, const HaldexControlConfig& cfg, float rearOverrunSlipMps) {
    const float radToDeg = 57.29578f;

    if (slipDeviationRadS > cfg.balanceUndersteerThresholdRadS) {
        float rawUndersteerDelta = slipDeviationRadS - cfg.balanceUndersteerThresholdRadS;
        float deviationDegS = rawUndersteerDelta * radToDeg;

        return clamp(deviationDegS * activeConfig().chassisBalanceGain, 0.0f, cfg.balanceUndersteerMaxLock);
    }

    if (isOversteerActive(slipDeviationRadS)) {
        if (escOff) {
            return 0.0f;
        }

        float rawOversteerDelta = std::abs(slipDeviationRadS) - std::abs(cfg.balanceOversteerThresholdRadS);
        float deviationDegS = rawOversteerDelta * radToDeg;
        bool wheelConfirms = rearOverrunSlipMps > cfg.rearOverrunSlipThresholdMps;
        float gain = wheelConfirms ? cfg.oversteerDualConfirmGain : 1.0f;

        return -clamp(deviationDegS * activeConfig().chassisBalanceGain * gain, 0.0f, cfg.balanceOversteerMaxRelease);
    }

    return 0.0f;
}

float calculateParkingDegradation(float V, float steeringAngleRad) {
    if (V >= activeConfig().steeringReductionLowSpeedThreshold) {
        return 1.0f;
    }

    float tireAngle = std::abs(steeringAngleRad) / activeConfig().steeringRatio;

    if (tireAngle <= activeConfig().parkingDeactivationAngleRad) {
        return 1.0f;
    }

    float steeringFactor = clamp(1.0f - ((tireAngle - activeConfig().parkingDeactivationAngleRad) / (activeConfig().steeringReductionMaxAngleRad - activeConfig().parkingDeactivationAngleRad)), 0.0f, 1.0f);
    float speedFade = V / activeConfig().steeringReductionLowSpeedThreshold;

    return std::max(speedFade + (1.0f - speedFade) * std::max(steeringFactor, 0.10f), 0.10f);
}

float calculatePredictiveLocks(float V) {
    stateEstimationLayer.baseSpeedLock = calculateBaseSpeedLock(V);
    stateEstimationLayer.proactiveTorqueLock = calculateFeedforwardTorqueLock(stateEstimationLayer.anticipatedWheelTorqueNm);
    float feedforwardLock = clamp(stateEstimationLayer.baseSpeedLock + stateEstimationLayer.proactiveTorqueLock, 0.0f, activeConfig().maxFeedforwardLockCap);

    stateEstimationLayer.chassisBalanceAdjustment = calculateLateralChassisBalanceAdjustment(stateEstimationLayer.chassisSlipDeviationRadS, rawCanInput.escOff, activeConfig(), stateEstimationLayer.rearOverrunSlipMps);
    float lateralDynamicLock = stateEstimationLayer.lateralFeedbackLock + stateEstimationLayer.chassisBalanceAdjustment;
    bool oversteerEscOff = isOversteerActive(stateEstimationLayer.chassisSlipDeviationRadS) && rawCanInput.escOff;

    if (stateEstimationLayer.cornerEntryPredicted && !oversteerEscOff) {
        lateralDynamicLock += activeConfig().cornerEntryLockBoost * stateEstimationLayer.stabilityMarginPct;
    }

    float latCap = std::abs(activeConfig().maxLateralDynamicLockCap);
    lateralDynamicLock = clamp(lateralDynamicLock, -latCap, latCap);

    return feedforwardLock + lateralDynamicLock;
}

// ----------------------------------------------------------------------------
// Safety limits and output filter
// ----------------------------------------------------------------------------
float applyAbsSafetyOverride(float currentTargetLock, float V, float safeDt) {
    if (rawCanInput.absActive) {
        filterState.absBrakeEventEndTimer = activeConfig().absEventHoldoffS;

        if (!filterState.absBrakeEventActive) {
            filterState.absInitialLock = finalHaldexOutput.targetLockPct;
            filterState.absFadeLock = filterState.absInitialLock;
            filterState.absBrakeEventActive = true;
            filterState.absFadeInitialized = true;
        }
    } else {
        filterState.absBrakeEventEndTimer -= safeDt;

        if (filterState.absBrakeEventEndTimer <= 0.0f) {
            filterState.absBrakeEventActive = false;
            filterState.absFadeInitialized = false;
            filterState.absInitialLock = 0.0f;
        }

        return currentTargetLock;
    }

    float absRange = activeConfig().absSpeedHighThresholdMps - activeConfig().absSpeedLowThresholdMps;
    float speedFactor = (absRange > 0.01f) ? (V - activeConfig().absSpeedLowThresholdMps) / absRange : 1.0f;
    float speedScale = clamp(speedFactor, 0.0f, 1.0f);
    float absFadeFloor = filterState.absInitialLock * activeConfig().absMaxFloorRatio * speedScale;
    float alphaAbs = safeDt / (activeConfig().absFadeTauS + safeDt);
    filterState.absFadeLock += alphaAbs * (absFadeFloor - filterState.absFadeLock);

    return clamp(filterState.absFadeLock, 0.0f, 100.0f);
}

void applySlewRateFilter(float rawTargetLock, float safeDt) {
    float desiredChange = rawTargetLock - finalHaldexOutput.targetLockPct;
    float maxChangeUp = activeConfig().maxLockSlewRateUp * safeDt;
    float maxChangeDown = activeConfig().maxLockSlewRateDown * safeDt;

    if (desiredChange > 0.0f) {
        desiredChange = std::min(desiredChange, maxChangeUp);
    } else {
        desiredChange = std::max(desiredChange, -maxChangeDown);
    }

    finalHaldexOutput.targetLockPct = clamp(finalHaldexOutput.targetLockPct + desiredChange, 0.0f, 100.0f);
}

bool evaluateLaunchAndKickdown(float V, float safeDt) {
    if (filterState.launchControlHoldTimer > 0.0f) {
        filterState.launchControlHoldTimer -= safeDt;
    }

    if (rawCanInput.launchControlFlag) {
        filterState.launchFlagHoldoffTimer = activeConfig().launchFlagHoldoffS;
    } else if (filterState.launchFlagHoldoffTimer > 0.0f) {
        filterState.launchFlagHoldoffTimer -= safeDt;
    }

    bool debouncedLaunchFlag = rawCanInput.launchControlFlag || (filterState.launchFlagHoldoffTimer > 0.0f);
    bool launchConditionsMet = debouncedLaunchFlag && V < 2.0f && processedSignalsLayer.normalizedThrottle > 0.85f && std::abs(processedSignalsLayer.steeringAngleRad) < activeConfig().launchSteeringMaxColumnRad;

    if (launchConditionsMet) {
        filterState.launchControlHoldTimer = activeConfig().launchControlHoldTimeS;
    }

    stateEstimationLayer.launchActive = filterState.launchControlHoldTimer > 0.0f;

    return (stateEstimationLayer.launchActive || rawCanInput.kickdown);
}

// ----------------------------------------------------------------------------
// Main loop
// ----------------------------------------------------------------------------
float prepareCycleMetrics(float dtSeconds, float& safeDt, float& normalAttenuation) {
    safeDt = clamp(dtSeconds, 0.001f, 0.050f);
    updateDriveModeFromCan(rawCanInput);
    processSignalsFiltered(rawCanInput, processedSignalsLayer, filterState, safeDt);
    normalAttenuation = calculateOutputAttenuation(rawCanInput);

    return clamp(processedSignalsLayer.vehicleSpeedMps, 0.05f, 100.0f);
}

void haldexControllerExecutionTask(float dtSeconds) {
    float safeDt = 0.0f;
    float normalAttenuation = 0.0f;
    float V = prepareCycleMetrics(dtSeconds, safeDt, normalAttenuation);
    float lockAccumulator = 0.0f;

    if (!rawCanInput.reverseGear) {
        estimateSteeringAndYaw(processedSignalsLayer, filterState, stateEstimationLayer, V, safeDt);
        estimateWheelTorque(rawCanInput, physConfig, stateEstimationLayer);
        estimateReactiveSlip(rawCanInput, processedSignalsLayer, stateEstimationLayer, V);
        estimateLateralDynamics(rawCanInput, physConfig, processedSignalsLayer, stateEstimationLayer, V);
        lockAccumulator = calculatePredictiveLocks(V);
    } else {
        estimateWheelTorque(rawCanInput, physConfig, stateEstimationLayer);
        estimateReactiveSlip(rawCanInput, processedSignalsLayer, stateEstimationLayer, V);
        estimateLateralDynamics(rawCanInput, physConfig, processedSignalsLayer, stateEstimationLayer, V);

        stateEstimationLayer.baseSpeedLock = 0.0f;
        stateEstimationLayer.proactiveTorqueLock = 0.0f;
        stateEstimationLayer.chassisBalanceAdjustment = 0.0f;
        stateEstimationLayer.cornerEntryPredicted = false;

        float latCap = std::abs(activeConfig().maxLateralDynamicLockCap);
        lockAccumulator = clamp(stateEstimationLayer.lateralFeedbackLock, 0.0f, latCap);
    }

    bool hasSlip = (stateEstimationLayer.frontRearSlipMps > activeConfig().slipTriggerThresholdMps);
    float parkingFactor = hasSlip ? 1.0f : calculateParkingDegradation(V, processedSignalsLayer.steeringAngleRad);
    float gripScale = std::max(stateEstimationLayer.tireGripAvailablePct, activeConfig().predictiveLockGripFloor);
    lockAccumulator *= parkingFactor * gripScale;

    if (hasSlip) {
        float rawReactive = stateEstimationLayer.frontRearSlipMps * stateEstimationLayer.dynamicSlipGain;
        float reactiveCap = rawCanInput.reverseGear ? activeConfig().maxReverseReactiveSlipLockCap : activeConfig().maxReactiveSlipLockCap;
        stateEstimationLayer.reactiveSlipLock = clamp(rawReactive, 0.0f, reactiveCap);

        float reactiveGripScale = std::max(stateEstimationLayer.tireGripAvailablePct, activeConfig().reactiveMinGripFloor);
        lockAccumulator += stateEstimationLayer.reactiveSlipLock * reactiveGripScale;
    } else {
        stateEstimationLayer.reactiveSlipLock = 0.0f;
    }

    float rawTargetLock = clamp(lockAccumulator, 0.0f, 100.0f) * (1.0f - normalAttenuation);

    if (evaluateLaunchAndKickdown(V, safeDt) && !filterState.absBrakeEventActive) {
        rawTargetLock = 100.0f;
    }

    rawTargetLock = applyAbsSafetyOverride(rawTargetLock, V, safeDt);
    applySlewRateFilter(rawTargetLock, safeDt);
}
