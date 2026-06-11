#include "diagnostics.hpp"

#include "core/haldexController.hpp"

#include <cstdio>

void printActiveControlConfig() {
    const auto& cfg = activeConfig();
    const char* modeStr = (driveMode == DriveMode::SPORT) ? "SPORT" : "NORMAL";

    printf("\n======================================================================\n");
    printf("   HALDEX CONTROL CONFIGURATION TARGET [MODE: %s]\n", modeStr);
    printf("======================================================================\n");
    printf("  %-45s : %.2f %s\n", "Base Speed Lock (Low Speed)", cfg.baseLockLowSpeed, "%");
    printf("  %-45s : %.2f m/s (%.1f km/h)\n", "Base Lock High Speed Cut-off", cfg.baseLockHighSpeedCutMps, cfg.baseLockHighSpeedCutMps * 3.6f);
    printf("  %-45s : %.2f\n", "Throttle Proactive Gain", cfg.throttleProactiveGain);
    printf("  %-45s : %.2f\n", "Slip Reactive Gain", cfg.slipReactiveGain);
    printf("  %-45s : %.2f\n", "Steering Ratio", cfg.steeringRatio);
    printf("  %-45s : %s\n", "Steering Input Type", cfg.steeringInputIsColumnAngle ? "Column angle (steering wheel)" : "Tire angle (wheels)");
    printf("  %-45s : %.2f s\n", "Launch Control Hold Time", cfg.launchControlHoldTimeS);
    printf("  %-45s : %.4f rad (%.1f deg column)\n", "Launch Max Steering (Column Angle)", cfg.launchSteeringMaxColumnRad, cfg.launchSteeringMaxColumnRad * 57.2958f);
    printf("  %-45s : %.3f m/s\n", "Slip Trigger Threshold", cfg.slipTriggerThresholdMps);
    printf("----------------------------------------------------------------------\n");
    printf("  %-45s : %.2f %s\n", "Max Feedforward Lock Cap (Base+Torque)", cfg.maxFeedforwardLockCap, "%");
    printf("  %-45s : %.2f %s\n", "Max Lateral Dynamic Lock Cap", cfg.maxLateralDynamicLockCap, "%");
    printf("  %-45s : %.2f %s\n", "Max Reactive Slip Lock Cap", cfg.maxReactiveSlipLockCap, "%");
    printf("----------------------------------------------------------------------\n");
    printf("  %-45s : +%.2f %s/s\n", "Max Lock Slew Rate UP", cfg.maxLockSlewRateUp, "%");
    printf("  %-45s : -%.2f %s/s\n", "Max Lock Slew Rate DOWN", cfg.maxLockSlewRateDown, "%");
    printf("----------------------------------------------------------------------\n");
    printf("  %-45s : %.2f\n", "Chassis Balance Gain", cfg.chassisBalanceGain);
    printf("  %-45s : %.2f rad/s\n", "Slalom Transition Threshold (Steer Rate)", cfg.slalomTransitionThresholdRadS);
    printf("  %-45s : %.4f rad/s\n", "Balance Understeer Threshold", cfg.balanceUndersteerThresholdRadS);
    printf("  %-45s : %.4f rad/s\n", "Balance Oversteer Threshold", cfg.balanceOversteerThresholdRadS);
    printf("  %-45s : %.2f %s\n", "Balance Understeer Max Lock Limit", cfg.balanceUndersteerMaxLock, "%");
    printf("  %-45s : %.2f %s\n", "Balance Oversteer Max Release Limit", cfg.balanceOversteerMaxRelease, "%");
    printf("----------------------------------------------------------------------\n");
    printf("  %-45s : %.2f m/s (%.1f km/h)\n", "ABS Low Speed Threshold", cfg.absSpeedLowThresholdMps, cfg.absSpeedLowThresholdMps * 3.6f);
    printf("  %-45s : %.2f m/s (%.1f km/h)\n", "ABS High Speed Threshold", cfg.absSpeedHighThresholdMps, cfg.absSpeedHighThresholdMps * 3.6f);
    printf("  %-45s : %.2f %s\n", "ABS Max Floor Ratio (Dynamic Base)", cfg.absMaxFloorRatio * 100.0f, "%");
    printf("----------------------------------------------------------------------\n");
    printf("  %-45s : %.3f s\n", "Signal Filter Time Constant (V, Steer)", cfg.signalFilterTimeConstant);
    printf("  %-45s : %.3f s\n", "Wheel Oscillation Filter Time Constant", cfg.wheelOscillationFilterTime);
    printf("  %-45s : %.3f s\n", "Steering Rate Filter Time Constant", cfg.steeringRateFilterTime);
    printf("----------------------------------------------------------------------\n");
    printf("  %-45s : %.4f\n", "Yaw Gain Linear Coeff", cfg.yawGainLinearCoeff);
    printf("  %-45s : %.5f\n", "Yaw Gain Quadratic Coeff", cfg.yawGainQuadraticCoeff);
    printf("  %-45s : %.3f s\n", "Chassis Lag Max Delay", cfg.chassisLagMaxS);
    printf("  %-45s : %.3f s\n", "Chassis Lag Min Delay", cfg.chassisLagMinS);
    printf("  %-45s : %.4f\n", "Chassis Lag Scale factor", cfg.chassisLagScale);
    printf("----------------------------------------------------------------------\n");
    printf("  %-45s : %.2f m/s (%.1f km/h)\n", "Parking Reduction Speed Limit", cfg.steeringReductionLowSpeedThreshold, cfg.steeringReductionLowSpeedThreshold * 3.6f);
    printf("  %-45s : %.4f rad\n", "Parking Reduction Max Angle", cfg.steeringReductionMaxAngleRad);
    printf("  %-45s : %.4f rad\n", "Parking Deactivation Angle Floor", cfg.parkingDeactivationAngleRad);
    printf("----------------------------------------------------------------------\n");
    printf("  %-45s : %.2f %s\n", "Corner Entry Lock Boost", cfg.cornerEntryLockBoost, "%");
    printf("  %-45s : %.3f s\n", "ABS Fade Time Constant", cfg.absFadeTauS);
    printf("  %-45s : %.1f bar\n", "Brake Attenuation Start Pressure", cfg.brakeAttenuationStartBar);
    printf("  %-45s : %.1f bar\n", "Brake Attenuation Range (full 100%%)", cfg.brakeAttenuationStartBar + cfg.brakeAttenuationRangeBar);
    printf("======================================================================\n\n");
}

void printVehiclePhysicsConfig() {
    printf("======================================================================\n");
    printf("   VEHICLE MECHANICAL & PHYSICS CONFIGURATION\n");
    printf("======================================================================\n");
    printf("  %-45s : %.1f kg\n", "Vehicle Curb Mass", physConfig.massKg);
    printf("  %-45s : %.1f Nm\n", "Engine Maximum Torque Output", physConfig.engineMaxTorque);
    printf("  %-45s : %.1f %s\n", "Drivetrain Total Efficiency", physConfig.drivetrainEfficiency * 100.0f, "%");
    printf("  %-45s : %.2f m\n", "Wheelbase Length", physConfig.wheelbase);
    printf("  %-45s : %.5f\n", "Understeer Gradient (K-factor)", physConfig.understeerGradient);
    printf("  %-45s : %.2f G\n", "Tire Maximum Lateral Grip Limit", physConfig.tireMaxLateralG);
    printf("  %-45s : %.2f G\n", "Tire Maximum Longitudinal Grip Limit", physConfig.tireMaxLongG);
    printf("----------------------------------------------------------------------\n");
    printf("  %-45s : %.3f / %.3f\n", "Final Drive Ratios (Axle 1 / Axle 2)", physConfig.finalDrive1, physConfig.finalDrive2);
    printf("  Gear Ratios Grid:\n");

    for (int i = 1; i <= 7; i++) {
        printf("    %-43s [Gear %d] : %.3f\n", "", i, physConfig.gearRatios[i]);
    }

    printf("======================================================================\n\n");
}
