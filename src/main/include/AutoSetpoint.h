#pragma once
#include "PoseTypes.h"

// pose, armPose, useLimelight, driveRate, rotationRate
struct AutoSetpoint
{
    Pose pose;
    ArmPose armPose = ArmPose{Vector{-9, 9.75}, false, 0};
    bool useLimelight = false;
    double driveRate = .25;
    double rotationRate = 0.25;
};