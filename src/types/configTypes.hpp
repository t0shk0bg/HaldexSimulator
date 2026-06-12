#pragma once

enum class DriveMode : int {
    NORMAL = 0,
    SPORT = 1,
    COUNT // Number of modes — used only for array sizing, not a valid mode
};

struct HaldexControlConfig {
    // --- BASE AND DYNAMIC LOCK PARAMETERS ---

    // Base lock at 0 m/s [%]. At launch and low speed the Haldex is pre-engaged.
    float baseLockLowSpeed = 35.0f;

    // Speed [m/s = 95 km/h] at which the base lock fades linearly/exponentially to 0%.
    // Above this value the base lock component does not contribute — the vehicle is stable on the highway.
    float baseLockHighSpeedCutMps = 26.38f;

    // Exponent of the power law for fading out the base lock as speed increases.
    // 1.0 = linear, 2.0 = quadratic (softer start, sharper fade-out higher up).
    float baseLockSpeedExponent = 2.0f;

    // Maximum feedforward lock [%] at full engine torque (throttle to the floor).
    // Proactive component — adds lock before real slip appears.
    float throttleProactiveGain = 60.0f;

    // Reactive slip gain [%/(m/s)]. For every 1 m/s front/rear wheel difference
    // 12% lock is added (before grip scaling and the cap).
    float slipReactiveGain = 12.0f;

    // Steering system ratio [column/tire].
    // Converts the steering wheel angle into the real front tire angle for the kinematic yaw model.
    float steeringRatio = 14.5f;

    // Declares what angle the CAN layer provides in steeringAngleDeg.
    // true  = steering wheel angle (steering column) — standard for most MQB implementations.
    // false = front wheel angle (tire angle) — the controller multiplies by steeringRatio before processing.
    bool steeringInputIsColumnAngle = true;

    // Time [s] for which 100% lock is held after launch control,
    // even if the flag has already dropped. Keeps full AWD engagement on a standing start.
    float launchControlHoldTimeS = 1.5f;

    // Debounce/holdoff filter [s] for the launch control flag from the CAN bus.
    // Prevents false triggering on noisy or flickering signals.
    float launchFlagHoldoffS = 0.10f;

    // Maximum steering wheel angle (steering column angle) [rad] while launch control is active.
    // Note: the value is a column angle, NOT a tire angle (unlike the parking parameters).
    // Default 0.52f ≈ ±30° wheel → ±2.07° tire at steeringRatio = 14.5.
    // Parking mode starts at 0.10 rad tire angle (5.7°) — no conflict with this value.
    float launchSteeringMaxColumnRad = 0.52f;


    // --- SLIP SENSITIVITY TUNING ---

    // Minimum front/rear wheel speed difference [m/s = 5 cm/s] to activate
    // the reactive slip component. Below the threshold — zero contribution (deadband).
    float slipTriggerThresholdMps = 0.05f;

    // Minimum multiplier of slipReactiveGain at low wheelDataConfidence or weak grip.
    // Does not allow the reactive lock to drop below 40% of the nominal value.
    float slipGainMultiplierMin = 0.40f;

    // Maximum multiplier of slipReactiveGain with an excellent signal and high grip.
    // At 100% confidence — up to 150% of the nominal slip gain.
    float slipGainMultiplierMax = 1.50f;


    // --- ANTI-WINDUP BUDGETS AND LIMITS IN % ---

    // Upper cap of the feedforward (proactive throttle) lock component [%].
    // The throttle pedal alone cannot impose more than 60% lock.
    float maxFeedforwardLockCap = 60.0f;

    // Upper cap of the lateral dynamic lock component [%].
    // Corrections from yaw error and understeer/oversteer balance are limited to ±30%.
    float maxLateralDynamicLockCap = 30.0f;

    // Upper cap of the reactive slip lock component [%].
    // Even at very strong slip the reactive response does not exceed 50%.
    float maxReactiveSlipLockCap = 50.0f;

    // Upper cap of the reactive slip lock in REVERSE [%].
    // Higher than the forward cap (50%) — in reverse on ice/gravel more aggressive AWD may be needed.
    // Grip scaling limits naturally: on ice (grip ≈ 1.0, little g) → full cap reachable at large slip.
    // On hard tarmac (grip 0.65) → the cap scales down to ~55%.
    float maxReverseReactiveSlipLockCap = 85.0f;


    // --- TEMPORAL STABILITY / SLEW RATE ---

    // Maximum lock ramp-up rate [%/s].
    // At 10 Hz = 6.67%/cycle — fast engagement when traction is needed.
    float maxLockSlewRateUp = 666.67f;

    // Maximum lock ramp-down rate [%/s].
    // At 10 Hz = 1%/cycle — slow, smooth release for comfort and stability.
    float maxLockSlewRateDown = 100.0f;


    // --- DYNAMIC CORRECTION GAINS ---

    // Gain of the chassis balance correction [%/(rad/s)].
    // Scales the influence of the yaw error (real - expected yaw) on the lock.
    float chassisBalanceGain = 2.0f;

    // Threshold of the yaw rate change rate [rad/s²] for slalom detection.
    // Above this rate of change the controller reduces the correction,
    // so it does not "fight" the steering wheel during fast direction changes.
    float slalomTransitionThresholdRadS = 2.0f;


    // --- LATERAL DYNAMIC BALANCE ---

    // Yaw error threshold [rad/s ≈ 3°/s] for understeer detection.
    // On a positive error above the threshold — lock is added for better rotation.
    float balanceUndersteerThresholdRadS = 0.0523f;

    // Yaw error threshold [rad/s ≈ −4°/s] for oversteer detection.
    // On an error below the threshold — the lock is reduced for stabilization.
    float balanceOversteerThresholdRadS = -0.0698f;

    // Maximum lock added [%] at full understeer.
    // Upper bound of the understeer correction component.
    float balanceUndersteerMaxLock = 30.0f;

    // Maximum lock reduction [%] at full oversteer.
    // Upper bound of the release during oversteer correction.
    float balanceOversteerMaxRelease = 30.0f;

    // Rear overrun slip threshold [m/s] — the rear wheels spin slower than the front.
    // Early indication of oversteer; when exceeded it accelerates the lock release.
    float rearOverrunSlipThresholdMps = 0.15f;

    // Release multiplier on dual confirmation of oversteer
    // (yaw error + rear overrun at the same time). More aggressive reaction on confirmed oversteer.
    float oversteerDualConfirmGain = 1.5f;


    // --- ADAPTIVE ABS PARAMETERS BY SPEED ---

    // Lower speed bound [m/s = 40 km/h] for ABS fade interpolation.
    // Below this speed the floor ratio is at its maximum (absMaxFloorRatio).
    float absSpeedLowThresholdMps = 11.11f;

    // Upper speed bound [m/s = 120 km/h] for ABS fade interpolation.
    // Above this speed the floor ratio tends toward the minimum (keeps less lock at high speed).
    float absSpeedHighThresholdMps = 33.33f;

    // Minimum fraction of the initial lock kept during ABS activation [0–1].
    // Keeps partial AWD even with ABS active — avoids fully releasing the rear axle.
    float absMaxFloorRatio = 0.35f;


    // --- SIGNAL FILTERING ---

    // Time constant [s] of the IIR low-pass filter for the vehicle speed and the steering angle.
    // Larger value = stronger smoothing, but larger lag.
    float signalFilterTimeConstant = 0.02f;

    // Time constant [s] for filtering the micro-oscillations of the wheel speed.
    // Measures sensor noise; at high noise it reduces wheelDataConfidence.
    float wheelOscillationFilterTime = 0.023f;

    // Time constant [s] for filtering the derivative of the steering angle (steering rate).
    // Provides a smooth and stable signal for the turning rate.
    float steeringRateFilterTime = 0.020f;

    // Time constant [s] for filtering the derivative of the yaw rate (yaw acceleration).
    // Kept short: the MK100 IPB already HW-conditions yaw (RMS ~0.1-0.2 deg/s), so heavy
    // smoothing is unnecessary and would add lag to this leading-indicator signal.
    // Diagnostic only for now: yaw acceleration is computed and logged, not yet used in control.
    float yawAccelFilterTime = 0.012f;


    // --- KINEMATIC GAINS (YAW) ---

    // Linear coefficient for the speed-dependent yaw gain.
    // gain = 1 + linear × V + quadratic × V² — corrects the expected yaw rate at high speed.
    float yawGainLinearCoeff = 0.054f;

    // Quadratic coefficient for the yaw gain — compensates the nonlinear tire behavior
    // (reduced grip at high speed leads to lower real yaw sensitivity).
    float yawGainQuadraticCoeff = 0.0025f;


    // --- ADAPTIVE INERTIAL LAGS ---

    // Maximum chassis inertial lag [s] at low speed.
    // Compensates the physical delay between command and the Haldex mechanical response.
    float chassisLagMaxS = 0.180f;

    // Minimum inertial lag [s] at high speed.
    // At high speed the mechanics react faster; the lag is reduced automatically.
    float chassisLagMinS = 0.045f;

    // Speed scale [1/m] for interpolating the chassis lag between min and max.
    // lag = maxS × exp(−scale × V) — determines how fast the lag decreases with V.
    float chassisLagScale = 0.0144f;


    // --- LOW-SPEED PARKING MODE ---

    // Speed bound [m/s = 20 km/h] below which the parking mode activates.
    // When maneuvering at low speed the lock is reduced to protect the Haldex.
    float steeringReductionLowSpeedThreshold = 5.556f;

    // Front tire angle [rad ≈ 30°] at full parking lock reduction.
    // At a larger angle lock = 0% — prevents overheating during slow full-lock turns.
    float steeringReductionMaxAngleRad = 0.523f;

    // Minimum tire angle [rad ≈ 5.7°] for any parking reduction to be applied.
    // Below the threshold — the lock is normal (small parking corrections do not count).
    float parkingDeactivationAngleRad = 0.10f;


    // --- GRIP SCALING OF THE REACTIVE SLIP ---

    // Minimum multiplier of the reactive lock at very low grip [0–1 = 15% minimum].
    // Even on ice or snow the reactive component keeps at least 15% of the nominal.
    float reactiveMinGripFloor = 0.15f;


    // --- GRIP SCALING OF PREDICTIVE LOCKS ---

    // Minimum multiplier of the feedforward/lateral locks at very low grip [0–1 = 8% minimum].
    // Keeps minimal proactivity even at near-zero grip — does not fully disable AWD.
    float predictiveLockGripFloor = 0.08f;


    // --- CORNER ENTRY BOOST ---

    // Additional lock [%] on detected corner entry (corner entry predicted).
    // Pre-engages the Haldex before slip appears for better rotation and traction.
    float cornerEntryLockBoost = 25.0f;


    // --- ABS FADE TIMING ---

    // Time constant [s] of the exponential lock fade during ABS.
    // lock(t) = floor + (initial − floor) × exp(−t / tau). Smaller value = faster fade.
    float absFadeTauS = 0.060f;

    // Holdoff [s] after the last ABS pulse before the end of the brake event.
    // Until the timer expires the absInitialLock snapshot is kept — prevents ratcheting on a rapidly pulsing ABS.
    float absEventHoldoffS = 0.300f;


    // --- BRAKE ATTENUATION ---

    // Brake pressure [bar] from which the AWD lock reduction begins.
    // Below this value the brakes do not affect the Haldex (normal operation).
    float brakeAttenuationStartBar = 15.0f;

    // Brake pressure range [bar] for full lock attenuation.
    // At pressure ≥ start + range the lock is 0% — prevents locking up while braking.
    float brakeAttenuationRangeBar = 45.0f;
};

struct HaldexCalibration {
    HaldexControlConfig configs[static_cast<int>(DriveMode::COUNT)];
};
