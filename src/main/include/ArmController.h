#pragma once
#include "PoseTypes.h"
#include "rev/CANSparkMax.h"
#include "ctre/phoenix.h"
#include "frc/Solenoid.h"
#include "frc/DigitalInput.h"


class ArmController
{
private:
    
    bool suctionCupState = false;

    // axis limits
    const double j1_Min = 0;
    const double j1_Max = 14.25;
    const double j2_Min = -3.2;
    const double j2_Max = 20;
    double j4_Max;
    double j4_Min;
    // position setpoint limits
    const double deck = -38.75;
    const double frame = 26;
    const double floor = -45.5;
    Vector origin = {19.75, -49.5};
    Vector startPosition = {10.75, -39.75};
    Vector currentPosition;
    Vector targetPosition;
    Vector error;
    double positionRate = 0.8;
    double j3Rate = 4;
    double j4Rate = 4;
    double j2Length;
    double startingJ1Length;
    double j1SetpointInches;
    double j3Current = 0;
    double j3Setpoint = 0;
    double j3Error;
    double j4UserSetpoint = 0;
    double j4Setpoint;
    double j4Current = 0;
    double startingJ4Position;
    double j4Error;

public:
    // leadscrew motors and PID controllers
    rev::CANSparkMax left_J1_NEO{41, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax right_J1_NEO{42, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkMaxPIDController left_J1 = left_J1_NEO.GetPIDController();
    rev::SparkMaxPIDController right_J1 = right_J1_NEO.GetPIDController();

    // arm extension motor and PID controller
    rev::CANSparkMax j2_NEO{43, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkMaxPIDController j2 = j2_NEO.GetPIDController();

    // end-of-arm motors and PID controllers
    rev::CANSparkMax j3_NEO{44, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax j4_NEO{45, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkMaxPIDController j3 = j3_NEO.GetPIDController();
    rev::SparkMaxPIDController j4 = j4_NEO.GetPIDController();

    // vacuum pumps
    WPI_TalonSRX pump1{51};
    WPI_TalonSRX pump2{52};
    frc::DigitalInput pressure1{0}; // right pressure switch digital input
    frc::DigitalInput pressure2{1}; // left pressure switch digital input
    frc::Solenoid suctionCup1{frc::PneumaticsModuleType::REVPH, 0};
    frc::Solenoid suctionCup2{frc::PneumaticsModuleType::REVPH, 15};

    ArmController()
    {
        currentPosition = startPosition;
        targetPosition = startPosition;
        startingJ4Position = angle(startPosition) - 90;
        startingJ1Length = sqrt(356 - 4 * sqrt(7345) * cos(atan2(startPosition.getX(), -startPosition.getY()) + atan(1.5 / 15.5) - atan(3.0 / 11)));
        j4_Max = 90 - startingJ4Position;
        j4_Min = j4_Max - 180;
    }

    void run(bool enabled, Vector velocityTarget = {}, double j3Velocity = 0, double j4Velocity = 0)
    {

        /* ------------- limits the target position ------------- */
        targetPosition += velocityTarget;
        if ((targetPosition.getY() < deck) && (targetPosition.getY() > floor) && (targetPosition.getX() < frame))
        {
            if (std::abs(targetPosition.getY() - deck) < std::abs(targetPosition.getX() - frame))
            {
                targetPosition.setY(deck);
            }
            else
            {
                targetPosition.setX(frame);
            }
        }
        else if ((targetPosition.getY() < floor) && (targetPosition.getX() > frame))
        {
            targetPosition.setY(floor);
        }
        else if ((targetPosition.getY() <= floor) && (targetPosition.getX() <= frame))
        {
            targetPosition = Vector{frame, floor};
        }


        if (enabled) {
            /* -- moves the currentPosition toward the target currentPosition -- */
            error = targetPosition - currentPosition;
            if (error > 2 * positionRate)
            {
                currentPosition += error * positionRate / abs(error);
            }
            else
            {
                currentPosition += error * 0.5;
            }

            /* ------------- limits the current position ------------- */
            if ((currentPosition.getY() < deck) && (currentPosition.getY() > floor) && (currentPosition.getX() < frame))
            {
                if (std::abs(currentPosition.getY() - deck) < std::abs(currentPosition.getX() - frame))
                {
                    currentPosition.setY(deck);
                }
                else
                {
                    currentPosition.setX(frame);
                }
            }
            else if ((currentPosition.getY() < floor) && (currentPosition.getX() > frame))
            {
                currentPosition.setY(floor);
            }
            else if ((currentPosition.getY() <= floor) && (currentPosition.getX() <= frame))
            {
                currentPosition = Vector{frame, floor};
            }

            /* ------------- set j4 position ------------- */
            j4UserSetpoint += j4Velocity;
            j4Setpoint = angle(currentPosition) - 90 - startingJ4Position + j4UserSetpoint;
            if (j4Setpoint > j4_Max)
            {
                j4UserSetpoint -= j4Setpoint - j4_Max;
                j4Setpoint = j4_Max;
            }
            if (j4Setpoint < j4_Min)
            {
                j4UserSetpoint += j4_Min - j4Setpoint;
                j4Setpoint = j4_Min;
            }

            /* --------- integrate toward new j4 position --------- */
            j4Error = j4Setpoint - j4Current;
            if (std::abs(j4Error) > 2 * j4Rate)
            {
                j4Current += j4Rate * j4Error / std::abs(j4Error);
            }
            else
            {
                j4Current += j4Error / 2;
            }

            /* ------------- set j3 position ------------- */
            j3Setpoint += j3Velocity;
            if (currentPosition.getX() < 26)
            {
                j3Setpoint = 0;
            }
            if (j3Setpoint > 90)
            {
                j3Setpoint = 90;
            }
            if (j3Setpoint < -90)
            {
                j3Setpoint = -90;
            }

            /* ------------- integrate toward new j3 position ------------- */
            j3Error = j3Setpoint - j3Current;
            if (std::abs(j3Error) > j3Rate)
            {
                j3Current += j3Rate * j3Error / std::abs(j3Error);
            }
            else
            {
                j3Current += j3Error / 2;
            }
            left_J1.SetReference(GetJ1(), rev::CANSparkMax::ControlType::kPosition);
            right_J1.SetReference(GetJ1(), rev::CANSparkMax::ControlType::kPosition);
            j2.SetReference(GetJ2(), rev::CANSparkMax::ControlType::kPosition);
            j3.SetReference(GetJ3(), rev::CANSparkMax::ControlType::kPosition);
            j4.SetReference(GetJ4(), rev::CANSparkMax::ControlType::kPosition);
        }
        pump1.Set((pressure1.Get()) ? 0.75 : 0);
        pump2.Set((pressure2.Get()) ? 0.75 : 0);
        suctionCup1.Set(suctionCupState);
        suctionCup2.Set(suctionCupState);
    }

    double GetJ1()
    {
        j1SetpointInches = sqrt(356 - 4 * sqrt(7345) * cos(atan2(currentPosition.getX(), -currentPosition.getY()) + atan(1.5 / 15.5) - atan(3.0 / 11))) - startingJ1Length;
        j1SetpointInches = (j1SetpointInches > j1_Max) ? j1_Max : (j1SetpointInches < j1_Min) ? j1_Min
                                                                                              : j1SetpointInches;
        return j1SetpointInches * (50.0 / 7);
    }

    double GetJ2()
    {
        j2Length = abs(currentPosition) - abs(startPosition);
        j2Length = (j2Length > j2_Max) ? j2_Max : (j2Length < j2_Min) ? j2_Min
                                                                      : j2Length;
        return j2Length * (5 * 25.4 / 18);
    }

    double GetJ3()
    {
        return j3Current * (5.0 / 36);
    }

    double GetJ4()
    {
        return j4Current * (5.0 / 36);
    }

    bool GetSuctionCupState() {
        return suctionCupState;
    }

    void toggleCupState() {
        suctionCupState = !suctionCupState;
    }

    void setArmPosition(ArmPose armPose = {}, bool isAutonomous = false)
    {
        targetPosition = origin + armPose.getPosition();
        j4UserSetpoint = armPose.getWrist();
        j3Setpoint = armPose.getTwist();
        if (isAutonomous)
        {
            suctionCupState = armPose.getSuctionCupState();
        }
    }

    bool poseReached(double tolerance)
    {
        return error < tolerance && j4Error < tolerance;
    }
};
