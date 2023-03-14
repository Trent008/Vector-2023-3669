#pragma once
#include "Parameters.h"
// pose, armPosition, wrist, suction, useLimelight, driveRate, rotationRate
struct AutoSetpoint
{
    Pose pose = params.startingPose;
    ArmPose armPose = ArmPose{Vector{-9, 9.75}, 0};
    bool suctionCupState = false;
    bool useLimelight = false;
    double driveRate = 15;
    double rotationRate = 15;
};