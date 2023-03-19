#pragma once
#include "Pose.h"

// pose, armPose, useLimelight, driveRate, rotationRate
struct AutoSetpoint
{
    Pose pose;
    Pose armPose = Pose{Vector{-9, 9.75}, 0};
    bool cupState = false;
    bool useLimelight = false;
    double driveRate = .25;
    double rotationRate = 0.25;
};