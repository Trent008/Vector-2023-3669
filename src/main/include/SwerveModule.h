#pragma once
#include "ctre/phoenix.h"
#include "Vector.h"
#include "Angle.h"
#include "rev/CANSparkMax.h"

class SwerveModule
{
private:
    Vector turnVector;         // vector corresponding to the way the rotation rate adds to the swerve module velocity
    double wheelDirection = 0; // direction of the wheel depending on whether the wheel is drivinging forward or backward
    double wheelSpeed = 0;
    Angle error = 0;
    double steeringMotorP; // proportional value determines how quickly the steering responds to angle setpoints
    Angle angleWheel, angleSetpoint;
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
    SwerveModule(WPI_TalonFX *driveMotor, rev::CANSparkMax *steeringMotor, CANCoder *wheelEncoder, Vector position)
    {
        steeringMotorP = 1;
        this->driveMotor = driveMotor;
        this->steeringMotor = steeringMotor;
        this->wheelEncoder = wheelEncoder;
        turnVector = position;
        turnVector /= abs(turnVector);
        turnVector += Angle{90};
    }

    // returns the angle the wheel needs to turn to
    void findSpeedAndAngleError(Vector velocity)
    {
        
        error = angle(velocity) - wheelEncoder->GetAbsolutePosition();
        if (abs(error) < 90) {
            wheelDirection = 1;
        }
        else
        {
            error += 180;
            wheelDirection = -1;
        }
        wheelSpeed = wheelDirection * abs(velocity);
    }

    void Set(Vector velocity)
    {
        findSpeedAndAngleError(velocity);
        driveMotor->Set(wheelSpeed);
        steeringMotor->Set(error.getValue() * -steeringMotorP / 180);
        currentPosition = driveMotor->GetSelectedSensorPosition(0);
        wheelPositionChange = Vector{0, currentPosition - lastPosition} + angle(velocity);
        lastPosition = currentPosition;
    }

    Vector getTurnVector()
    {
        return turnVector;
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