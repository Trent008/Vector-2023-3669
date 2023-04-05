#pragma once
#include "AutoSetpoint.h"

// scroll to bottom for autonomous setpoints

Pose armPresets[8][2] =
    {
        {{{-9, 11}, 8}, {{-9, 14}, 8}}, // home
        {{{8, 9}}, {{8, 8}, -7}},   // low
        {{{13, 30}, 0}, {{20, 38.5}, 20}}, // mid
        {{{33, 45}, 0}, {{37, 54}, 0}}, // high
        {{{5, 16}, 0}, {{8, 18}, 10}}, // intermediate
        {{{5, 16}, -90}, {{8, 6}, -7}}, // floor
        {{{15, 43}, 0}, {{15, 44}, -7}},  // feeder station
        {{{15, 43}, 0}, {{15, 50}, 0}}  // intermediate for placing
};

// parameters for robot movement and autonomous
struct Parameters
{
    // set ramp
    double robotAccel = 0.03;     // acceleration rate of the robot pose on the field
    bool isAutonomous;

    Pose drop = {{0, -5}, 0}; //{0, -7}

    // swerve presets
    Pose p1 = {{75, 96}, -90}; //{75, 18.5};
    Pose offset = {{20}, 0};
    Pose chargingStation = {{152, 96}, -90};
    Pose startingPose = p1;

    /*
     * the pose, armPosition, wrist, suction, useLimelight, driveRate, rotationRate
     * for the autonomous routine
     */
    AutoSetpoint setpoints[15] =
        {
            {p1, armPresets[7][1] + Pose{{0, 0}, 80}, true},
            {p1, armPresets[3][1], true},
            {p1, armPresets[3][1] + drop, true},
            {chargingStation + Pose{{70}}, armPresets[0][1], false},
            {chargingStation - Pose{{7}}, armPresets[0][1], false},
            {chargingStation - Pose{{7}}, armPresets[0][1], false},
            {chargingStation - Pose{{7}}, armPresets[0][1], false},
            {chargingStation - Pose{{7}}, armPresets[0][1], false},
            {chargingStation - Pose{{7}}, armPresets[0][1], false},
            {chargingStation - Pose{{7}}, armPresets[0][1], false},
            {chargingStation - Pose{{7}}, armPresets[0][1], false},
            {chargingStation - Pose{{7}}, armPresets[0][1], false},
            {chargingStation - Pose{{7}}, armPresets[0][1], false},
            {chargingStation - Pose{{7}}, armPresets[0][1], false},
            {chargingStation - Pose{{7}}, armPresets[0][1], false}
    };
} params;