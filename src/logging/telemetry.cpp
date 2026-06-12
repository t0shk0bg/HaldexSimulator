#include "telemetry.hpp"

#include "core/haldexController.hpp"

#include <cstdio>

void sendHaldexTelemetryHeader() {
    printf("LOCK_PCT,SLIP_DEV,CORNER,SLIP_MPS,REAR_OVRRN,ABS_FADE,DT,TORQUE_NM,"
           "CONF,GRIP,BRAKE_P,BASE_LOCK,PRO_LOCK,REACT_LOCK,BAL_ADJ,"
           "SPD_KMH,WFL,WFR,WRL,WRR,THROT,BRAKE,STEER,GEAR,TORQ_ACT,"
           "TORQ_REQ,MODE,ABS,ASR,ESC,LC,KD,BRAKEP2,YAW,LAT_G,LONG_G,ESC_OFF,YAW_ACC\n");
}

void sendHaldexTelemetry(float safeDt) {
    printf("%.1f,%.4f,%d,%.3f,%.3f,%.1f,%.4f,%.1f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%.1f,%d,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%.1f,%.1f,%.2f,%.2f,%d,%.3f\n",
           finalHaldexOutput.targetLockPct,                   // 1
           stateEstimationLayer.chassisSlipDeviationRadS,     // 2
           stateEstimationLayer.cornerEntryPredicted ? 1 : 0, // 3
           stateEstimationLayer.frontRearSlipMps,             // 4
           stateEstimationLayer.rearOverrunSlipMps,           // 5
           filterState.absFadeLock,                           // 6
           safeDt,                                            // 7
           stateEstimationLayer.anticipatedWheelTorqueNm,     // 8
           stateEstimationLayer.wheelDataConfidence,          // 9
           stateEstimationLayer.tireGripAvailablePct,         // 10
           rawCanInput.brakePressure,                         // 11
           stateEstimationLayer.baseSpeedLock,                // 12
           stateEstimationLayer.proactiveTorqueLock,          // 13
           stateEstimationLayer.reactiveSlipLock,             // 14
           stateEstimationLayer.chassisBalanceAdjustment,     // 15
           rawCanInput.vehicleSpeedKmh,                       // 16
           rawCanInput.wheelSpeedFl,                          // 17
           rawCanInput.wheelSpeedFr,                          // 18
           rawCanInput.wheelSpeedRl,                          // 19
           rawCanInput.wheelSpeedRr,                          // 20
           rawCanInput.throttlePct,                           // 21
           rawCanInput.brakePressed ? 1 : 0,                  // 22
           rawCanInput.steeringAngleDeg,                      // 23
           rawCanInput.gear,                                   // 24
           rawCanInput.actualTorqueSumNm,                     // 25
           rawCanInput.filteredRequestedTorqueNm,             // 26
           rawCanInput.selectedDriveMode,                     // 27
           rawCanInput.absActive ? 1 : 0,                     // 28
           rawCanInput.asrInterventionActive ? 1 : 0,         // 29
           rawCanInput.escInterventionActive ? 1 : 0,         // 30
           rawCanInput.launchControlFlag ? 1 : 0,             // 31
           rawCanInput.kickdown ? 1 : 0,                      // 32
           rawCanInput.brakePressure,                         // 33
           rawCanInput.yawRateDegS,                           // 34
           rawCanInput.lateralAccelG,                         // 35
           rawCanInput.longitudinalAccelG,                    // 36
           rawCanInput.escOff ? 1 : 0,                        // 37
           stateEstimationLayer.yawAccelRadS2                 // 38
    );
}
