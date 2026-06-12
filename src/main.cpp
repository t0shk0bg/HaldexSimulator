#include "core/haldexController.hpp"
#include "logging/diagnostics.hpp"
#include "logging/telemetry.hpp"

#include <cmath>

int main() {
    HaldexControlConfig normalCfg;
    HaldexControlConfig sportCfg;

    // --- SPORT: Base aggressiveness ---
    sportCfg.baseLockLowSpeed = 50.0f;
    sportCfg.baseLockSpeedExponent = 2.5f;
    sportCfg.throttleProactiveGain = 80.0f;
    sportCfg.chassisBalanceGain = 3.0f;
    sportCfg.maxLockSlewRateDown = 60.0f;
    sportCfg.cornerEntryLockBoost = 35.0f;

    // --- SPORT: Slip sensitivity ---
    sportCfg.parkingDeactivationAngleRad = 0.20f;
    sportCfg.slipGainMultiplierMax = 1.71f;
    sportCfg.reactiveMinGripFloor = 0.08f;
    sportCfg.predictiveLockGripFloor = 0.05f;

    // --- SPORT: Brake trail braking ---
    sportCfg.brakeAttenuationStartBar = 25.0f;

    setupHaldexControl({normalCfg, sportCfg});
    setupVehiclePhysics(VehiclePhysicsConfig{});

    printVehiclePhysicsConfig();
    printActiveControlConfig();
    sendHaldexTelemetryHeader();

    // Simulation loop: 100 Hz, 5 seconds
    const float dt = 0.010f;
    const int cycles = 200;

    for (int i = 0; i < cycles; i++) {
        float t = i * dt;

        rawCanInput.vehicleSpeedKmh = clamp(t * 24.0f, 0.0f, 120.0f);
        float spd = rawCanInput.vehicleSpeedKmh;

        rawCanInput.wheelSpeedFl = spd;
        rawCanInput.wheelSpeedFr = spd;
        rawCanInput.wheelSpeedRl = (i > 100) ? spd - 2.5f : spd;
        rawCanInput.wheelSpeedRr = (i > 100) ? spd - 2.5f : spd;

        rawCanInput.throttlePct = clamp(100.0f - t * 8.0f, 20.0f, 100.0f);
        rawCanInput.actualTorqueSumNm = rawCanInput.throttlePct * 3.5f;
        rawCanInput.filteredRequestedTorqueNm = rawCanInput.actualTorqueSumNm;

        rawCanInput.gear = (spd < 20.0f) ? 1 : (spd < 50.0f) ? 2
                                             : (spd < 90.0f)   ? 3
                                                               : 4;

        rawCanInput.steeringAngleDeg = 15.0f * std::sin(t * 0.5f);
        rawCanInput.yawRateDegS = 8.0f * std::sin(t * 0.5f);
        rawCanInput.lateralAccelG = 0.18f * std::sin(t * 0.5f);
        rawCanInput.longitudinalAccelG = 0.35f;

        rawCanInput.brakePressed = false;
        rawCanInput.brakePressure = 0.0f;
        rawCanInput.absActive = false;
        rawCanInput.asrInterventionActive = false;
        rawCanInput.escInterventionActive = false;
        rawCanInput.launchControlFlag = false;
        rawCanInput.kickdown = false;
        rawCanInput.escOff = false;
        rawCanInput.selectedDriveMode = 0;

        haldexControllerExecutionTask(dt);
        sendHaldexTelemetry(dt);
    }

    return 0;
}
