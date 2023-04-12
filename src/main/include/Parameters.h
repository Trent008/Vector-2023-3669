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
    double robotAccel = 0.04;     // acceleration rate of the robot pose on the field

    Pose drop = {{0, -5}, 0};

    // swerve presets
    Pose chargingStation = {{0, 77}, 180};
    Pose startingPose = {{0, 0}, 180};

    /*
     * the pose, armPosition, wrist, suction, useLimelight, driveRate, rotationRate
     * for the autonomous routine
     */
    AutoSetpoint setpoints[15] =
        {
            {startingPose, armPresets[7][1] + Pose{{0, 0}, 80}, true},
            {startingPose, armPresets[3][1], true},
            {startingPose, armPresets[3][1] + drop, true},
            {chargingStation + Pose{{92}}, armPresets[0][1], false},
            {chargingStation - Pose{{3}}, armPresets[0][1], false},
            {chargingStation - Pose{{3}}, armPresets[0][1], false},
            {chargingStation - Pose{{3}}, armPresets[0][1], false},
            {chargingStation - Pose{{3}}, armPresets[0][1], false},
            {chargingStation - Pose{{3}}, armPresets[0][1], false},
            {chargingStation - Pose{{3}}, armPresets[0][1], false},
            {chargingStation - Pose{{3}}, armPresets[0][1], false},
            {chargingStation - Pose{{3}}, armPresets[0][1], false},
            {chargingStation - Pose{{3}}, armPresets[0][1], false},
            {chargingStation - Pose{{3}}, armPresets[0][1], false},
            {chargingStation - Pose{{3}}, armPresets[0][1], false}
    };
} params;