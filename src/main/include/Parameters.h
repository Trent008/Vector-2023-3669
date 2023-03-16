#pragma once
#include "AutoSetpoint.h"

// scroll to bottom for autonomous setpoints

// home position for cone or cube
ArmPose home(bool isCone, bool isHoldingObject) {
    if (isCone)
    {
        return {{-9, 11}, isHoldingObject, 8, 0};
    }
    return {{-9, 11}, isHoldingObject, 8, 0};
}

// top pole or cube slot
ArmPose top(bool isCone) {
    if (isCone)
    {
        return {{35.5, 51}, true, 0, 0};
    }
    return {{30, 45}, true, 0, 0};
}

// lower pole or cube slot
ArmPose middle(bool isCone) {
    if (isCone)
    {
        return {{21, 40}, true, 0, 0};
    }
    return {{13, 30}, true, 0, 0};
}

// hybrid node for cone or cube
ArmPose bottom(bool isCone) {
    if (isCone)
    {
        return {{8, 4}, true, -7};
    }
    return {{8, 4}, true, -7};
}

// human feeder station for cone or cube
ArmPose feederStation(bool isCone) {
    if (isCone)
    {
        return {{15, 43}, true, 0, 0};
    }
    return {{15, 43}, true, 0, 0};
}

struct Parameters
{
    Pose startingPose = {Vector{75, 96}, -90};
    // set ramp
    double robotAccel = 0.03;     // acceleration rate of the robot speed on the field
    double robotTurnAccel = 0.03; // acceleration rate of robot steering rate

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
            {chargingStation + Vector{70}, home(1, 0), false},
            {chargingStation, home(1, 0), false},
            {chargingStation, home(1, 0), false},
            {chargingStation, home(1, 0), false},
            {chargingStation, home(1, 0), false},
            {chargingStation, home(1, 0), false},
            {chargingStation, home(1, 0), false},
            {chargingStation, home(1, 0), false},
            {chargingStation, home(1, 0), false},
            {chargingStation, home(1, 0), false},
            {chargingStation, home(1, 0), false},
            {chargingStation, home(1, 0), false},
    };
} params;