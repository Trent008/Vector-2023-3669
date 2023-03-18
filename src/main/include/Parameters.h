#pragma once
#include "AutoSetpoint.h"

// scroll to bottom for autonomous setpoints

// home position for cone or cube
ArmPose home(bool isCone, bool isHoldingObject = false) {
    if (isCone)
    {
        return ArmPose{{-9, 11}, 8, isHoldingObject};
    }
    return ArmPose{{-9, 11}, 8, isHoldingObject};
}

ArmPose floor(bool isCone) {
    if (isCone)
    {
        return ArmPose{{8, 6}, -7, true};
    }
    return ArmPose{{5, 16}, -90, true};
}

// top pole or cube slot
ArmPose top(bool isCone) {
    if (isCone)
    {
        return {{37, 54}, 0, true};
    }
    return {{33, 45}, 0, true};
}

// lower pole or cube slot
ArmPose middle(bool isCone) {
    if (isCone)
    {
        return {{21, 40}, 0, true};
    }
    return {{13, 30}, 0, true};
}

// hybrid node for cone or cube
ArmPose bottom(bool isCone) {
    if (isCone)
    {
        return {{8, 8}, -7, true};
    }
    return {{8, 8}, -7, true};
}

// human feeder station for cone or cube
ArmPose feederStation(bool isCone) {
    if (isCone)
    {
        return {{15, 44}, 0, true};
    }
    return {{15, 43}, 0, true};
}

// parameters for robot movement and autonomous
struct Parameters
{
    Pose startingPose = {Vector{75, 96}, -90};
    // set ramp
    double robotAccel = 0.05;     // acceleration rate of the robot speed on the field
    double robotTurnAccel = 0.03; // acceleration rate of robot steering rate
    bool isAutonomous;

    Vector drop = {0, -5}; //{0, -7}

    // swerve presets
    Pose p1 = {{75, 96}, -90}; //{75, 18.5};
    Vector offset = {20};
    Pose chargingStation = {{152, 96}, -90};

    /*
     * the pose, armPosition, wrist, suction, useLimelight, driveRate, rotationRate
     * for the autonomous routine
     */
    AutoSetpoint setpoints[15] =
        {
            {p1 + offset, top(1), false},
            {p1, top(1), false},
            {p1, top(1) + drop, false},
            {chargingStation + Vector{70}, home(1), false},
            {chargingStation, home(1), false},
            {chargingStation, home(1), false},
            {chargingStation, home(1), false},
            {chargingStation, home(1), false},
            {chargingStation, home(1), false},
            {chargingStation, home(1), false},
            {chargingStation, home(1), false},
            {chargingStation, home(1), false},
            {chargingStation, home(1), false},
            {chargingStation, home(1), false},
            {chargingStation, home(1), false},
    };
} params;