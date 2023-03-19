#pragma once
#include "AutoSetpoint.h"

// scroll to bottom for autonomous setpoints

Pose armPresets[7][2] =
    {
        {{{-9, 11}, 8}, {{-9, 11}, 8}}, // home
        {{{8, 8}, -7}, {{8, 8}, -7}},   // low
        {{{13, 30}, 0}, {{21, 40}, 0}}, // mid
        {{{33, 45}, 0}, {{37, 54}, 0}}, // high
        {{{5, 14}, 10}, {{8, 17}, 10}}, // intermediate
        {{{5, 16}, -90}, {{8, 6}, -7}}, // floor
        {{{15, 43}, 0}, {{15, 44}, 0}}  // feeder station
};

// parameters for robot movement and autonomous
struct Parameters
{
    Pose startingPose = {Vector{75, 96}, -90};
    // set ramp
    double robotAccel = 0.05;     // acceleration rate of the robot speed on the field
    double robotTurnAccel = 0.03; // acceleration rate of robot steering rate
    bool isAutonomous;

    Pose drop = {{0, -5}, 0}; //{0, -7}

    // swerve presets
    Pose p1 = {{75, 96}, -90}; //{75, 18.5};
    Pose offset = {{20}, 0};
    Pose chargingStation = {{152, 96}, -90};

    /*
     * the pose, armPosition, wrist, suction, useLimelight, driveRate, rotationRate
     * for the autonomous routine
     */
    AutoSetpoint setpoints[15] =
        {
            {p1 + offset, armPresets[3][1], true},
            {p1, armPresets[3][1], true},
            {p1, armPresets[3][1] + drop, true},
            {chargingStation + Pose{{70}}, armPresets[0][1], false},
            {chargingStation, armPresets[0][1], false},
            {chargingStation, armPresets[3][1], false},
            {chargingStation, armPresets[3][1], false},
            {chargingStation, armPresets[3][1], false},
            {chargingStation, armPresets[3][1], false},
            {chargingStation, armPresets[3][1], false},
            {chargingStation, armPresets[3][1], false},
            {chargingStation, armPresets[3][1], false},
            {chargingStation, armPresets[3][1], false},
            {chargingStation, armPresets[3][1], false},
            {chargingStation, armPresets[3][1], false},
    };
} params;