#pragma once
#include "SwerveDrive.h"

/**
 * allows the swerve drive to autonomously drive
 * to an array of current positions and angles
 **/
class RobotPoseTargeting
{
private:
    double positionProportional; // rate at which to approach the current position
    double angleProportional;    // rate at which to approach the current angle
    Pose poseError;              // how fast the robot needs to move to get to its next position setpoint
    Pose distanceToSetpointPose;
    Pose swerveRate;
    Pose current;
    SwerveDrive *swerve;

public:
    RobotPoseTargeting(SwerveDrive *swerve, double positionProportional, double angleProportional)
    {   
        this->positionProportional = positionProportional;
        this->angleProportional = angleProportional;
        this->swerve = swerve;
    }

    void targetPose(Pose setpoint, double driveRate, double rotationRate)
    {   
        //current.moveToward(setpoint, driveRate / 50, rotationRate / 50);
        //distanceToSetpointPose = setpoint - current;
        poseError = setpoint - swerve->getPose();
        swerveRate = poseError * Vector{positionProportional, angleProportional};
        swerveRate.limit(Vector{0.2, 0.2});
        swerve->Set(swerveRate, true);
    }

    bool poseReached(double positionTolerance, double angleTolerance)
    {
        return (poseError.getPosition() < positionTolerance) && (std::abs(distanceToSetpointPose.getAngle()) <= angleTolerance); //distanceToSetpointPose.getPosition()
    }

    void setCurrentPosition(Vector position) {
        //current.setPosition(position);
        swerve->setPosition(position);
    }


};