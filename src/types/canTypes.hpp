#pragma once

struct CanInputLayer {
    float vehicleSpeedKmh = 0.0f;
    float wheelSpeedFl = 0.0f;
    float wheelSpeedFr = 0.0f;
    float wheelSpeedRl = 0.0f;
    float wheelSpeedRr = 0.0f;
    float throttlePct = 0.0f;
    bool brakePressed = false;
    float steeringAngleDeg = 0.0f;
    int gear = 0;
    bool reverseGear = false;
    float actualTorqueSumNm = 0.0f;
    float filteredRequestedTorqueNm = 0.0f;
    int selectedDriveMode = 0;
    bool absActive = false;
    bool asrInterventionActive = false;
    bool escInterventionActive = false;
    bool launchControlFlag = false;
    bool kickdown = false;
    float brakePressure = 0.0f;
    float yawRateDegS = 0.0f;
    float lateralAccelG = 0.0f;
    float longitudinalAccelG = 0.0f;
    bool escOff = false;
};
