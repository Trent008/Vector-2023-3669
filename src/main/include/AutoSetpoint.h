#pragma once
#include "Parameters.h"
// pose, armPosition, suction, wrist, driveRate, rotationRate
struct AutoSetpoint
{
    Pose pose = params.startingPose;
    Vector armPosition = {-9, 9.75};
    bool suctionCupState = false;
    double wristAngle = 0;
    double driveRate = 15;
    double rotationRate = 15;
};