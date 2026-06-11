#pragma once

struct VehiclePhysicsConfig {
    // Vehicle mass [kg]. Used to compute load transfer under acceleration/braking,
    // which affects the load distribution between the axles.
    float massKg = 1550.0f;

    // Maximum engine torque [Nm]. Reference value for normalizing
    // throttleProactiveGain — at 400 Nm the maximum feedforward lock is reached.
    float engineMaxTorque = 400.0f;

    // Mechanical drivetrain efficiency [0–1]. Accounts for losses in the gearbox,
    // the prop shaft and the differentials when transferring torque to the wheels.
    float drivetrainEfficiency = 0.91f;

    // Gear ratios for 8 gears [—].
    // Index 0 = neutral/reverse (0.000, not used for torque).
    // Indices 1–7 = first to seventh forward gear.
    // Multiplied by final drive and the engine torque, they give the wheel torque.
    float gearRatios[8] = {0.000f, 3.563f, 2.526f, 1.679f, 1.022f, 0.788f, 0.761f, 0.634f};

    // Final drive ratio [—] — transverse configuration (front axle).
    // Multiplies the gearbox output before the front axle differential.
    float finalDrive1 = 4.059f;

    // Haldex/rear differential ratio [—].
    // Multiplies the rear axle torque when computing the expected wheel torque.
    float finalDrive2 = 3.130f;

    // Vehicle wheelbase [m] — the distance between the front and rear axles.
    // Used in the kinematic model: expected_yaw = (δ / ratio) × V / wheelbase.
    float wheelbase = 2.63f;

    // Understeer gradient [rad/g] — Marshall K-factor.
    // Positive value = tendency toward understeer as lateral acceleration increases.
    // Used to correct the expected yaw rate in steady-state cornering.
    float understeerGradient = 0.004f;

    // Maximum tire lateral acceleration [g] — the friction ellipse limit on the Y axis.
    // When this value is reached the longitudinal grip is sharply reduced (coupled tire behavior).
    float tireMaxLateralG = 1.05f;

    // Maximum tire longitudinal acceleration/deceleration [g] — the friction ellipse limit on the X axis.
    // Near this value under throttle or braking the lateral grip is strongly reduced.
    float tireMaxLongG = 0.95f;

    // Maximum wheel torque [Nm]. Filled automatically by setupVehiclePhysics()
    // as: engineMaxTorque × max(gearRatios) × finalDrive1 × finalDrive2 × drivetrainEfficiency.
    // Used to normalize anticipatedWheelTorqueNm to the 0–1 range.
    float maxWheelTorque = 0.0f;
};
