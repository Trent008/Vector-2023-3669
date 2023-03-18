#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include "cameraserver/CameraServer.h"
#include "SpaceMousePro.h"
#include "SpaceMouseEnt.h"
#include "XBOXController.h"
#include "ArmController.h"
#include "SwervePoseTargeting.h"
#include "Limelight.h"
#include "Parameters.h"

class Robot : public frc::TimedRobot
{

public:
  int i = 0; // keeps track of the autonomous point index
  int t = 0; // keeps track of the number of processer cycles

  Limelight limelight_left{"left"};
  Limelight limelight_right{"right"};

  frc::Joystick driveController{0};
  frc::Joystick armController{1};
  /* ------ driving controller types ------ */
  XBOXController xboxC{&driveController};
  SpaceMouseEnt SMEnt{&driveController};

  /* -------- arm controller types -------- */
  SpaceMousePro SMPro{&armController, 20};

  /* -------- swerve drive motors -------- */
  WPI_TalonFX driveMotor1{11};
  WPI_TalonFX driveMotor2{12};
  WPI_TalonFX driveMotor3{13};
  WPI_TalonFX driveMotor4{14};
  /* -------- swerve module encoders -------- */
  CANCoder encoder1{21};
  CANCoder encoder2{22};
  CANCoder encoder3{23};
  CANCoder encoder4{24};
  /* -------- swerve module wheel turning motors -------- */
  rev::CANSparkMax steeringMotor1{31, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax steeringMotor2{32, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax steeringMotor3{33, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax steeringMotor4{34, rev::CANSparkMax::MotorType::kBrushless};
  /* -------- swerve module objects -------- */
  SwerveModule *m1 = new SwerveModule{&driveMotor1, &steeringMotor1, &encoder1, {-.7, 1}};
  SwerveModule *m2 = new SwerveModule{&driveMotor2, &steeringMotor2, &encoder2, {-.7, -1}};
  SwerveModule *m3 = new SwerveModule{&driveMotor3, &steeringMotor3, &encoder3, {.7, 1}};
  SwerveModule *m4 = new SwerveModule{&driveMotor4, &steeringMotor4, &encoder4, {.7, -1}};
  // swerve module array:
  SwerveModule *modules[4] = {m1, m2, m3, m4};

  // field oriented motion control and motion smoothing class:
  FOC motionController{params.robotAccel, params.robotTurnAccel};
  // swerve drive object to control the 4-SwerveModule array using the motion controller object
  SwerveDrive swerve{&motionController, modules};
  SwervePoseTargeting swerveTargeting{&swerve, 0.03, 0.007};

  
  bool isCone = true;
  bool isHomingFromFloor = false;
  int armSetpointType = 0;

  ArmController arm{};

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

private:
  static void VisionThread()
  {
    // Get the USB camera from CameraServer
    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
    // Set the resolution
    camera.SetResolution(800, 600);
  }
};
