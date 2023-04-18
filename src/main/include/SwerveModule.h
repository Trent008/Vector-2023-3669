#pragma once
#include "ctre/phoenix.h"
#include "Pose.h"
#include "Angle.h"
#include "rev/CANSparkMax.h"

class SwerveModule
{
private:
    Vector turnVector;         // vector corresponding to the way the rotation rate adds to the swerve module velocity
    double wheelDirection; // direction of the wheel depending on whether the wheel is drivinging forward or backward
    double wheelSpeed;
    Angle error;
    double steeringMotorP; // proportional value determines how quickly the steering responds to angle setpoints
    double lastPosition = 0;
    double currentPosition;
    WPI_TalonFX *driveMotor;
    rev::CANSparkMax *steeringMotor;
    CANCoder *wheelEncoder;
    Vector wheelPositionChange;

public:
    /**
     * parameters posX and posY set the position of
     * the module relative to the center of the robot
     */
    SwerveModule(WPI_TalonFX *driveMotor, rev::CANSparkMax *steeringMotor, CANCoder *wheelEncoder, Vector position = {})
    {
        steeringMotorP = 1;
        this->driveMotor = driveMotor;
        this->steeringMotor = steeringMotor;
        this->wheelEncoder = wheelEncoder;
        turnVector = position;
        turnVector /= turnVector.magnitude();
        turnVector.rotateCW(90);
    }

    // returns the angle the wheel needs to turn to
    void findSpeedAndAngleError(Vector velocity)
    {
        error = Angle{velocity.getAngle()};
        error -= wheelEncoder->GetAbsolutePosition();
        if (error.magnitude() < 90) {
            wheelDirection = 1;
        }
        else
        {
            error += 180;
            wheelDirection = -1;
        }
        wheelSpeed = wheelDirection * velocity.magnitude();
    }

    Vector getVector(Pose robotRate)
    {
        return robotRate.getVector() + turnVector * robotRate.getAngle();
    }

    void Set(Vector velocity)
    {
        findSpeedAndAngleError(velocity);
        driveMotor->Set(wheelSpeed);
        steeringMotor->Set(error.get() * -steeringMotorP / 180);
        currentPosition = driveMotor->GetSelectedSensorPosition(0);
        wheelPositionChange = Vector{0, currentPosition - lastPosition};
        wheelPositionChange.rotateCW(wheelEncoder->GetAbsolutePosition());
        lastPosition = currentPosition;
    }

    double getWheelSpeed()
    {
        return wheelSpeed;
    }

    Vector getwheelPositionChange()
    {
        return wheelPositionChange;
    }
};