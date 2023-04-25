#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include "cameraserver/CameraServer.h"
#include "SpaceMouseEnt.h"
#include "Logitech.h"
#include "XBOXController.h"
#include "XKeysPad.h"
#include "ArmController.h"
#include "SwervePoseTargeting.h"

class Robot : public frc::TimedRobot
{

public:
  int i = 0; // keeps track of the autonomous point index
  int cycles = 0; // keeps track of the number of processer cycles

  frc::Joystick driveController{0};
  frc::Joystick armController{1};
  frc::Joystick pad{2};
  frc::Joystick wristController{3};

  /* ------ driving controller types ------ */
  XBOXController xboxC{&driveController};
  XKeysPad buttonPad{&pad};

  /* -------- arm controller types -------- */
  // SpaceMousePro SMPro{&armController, 20};
  SpaceMouseEnt SMEnt{&armController};
  Logitech logi{&armController};
  Logitech logi2{&wristController};


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

  // swerve module array:
  SwerveModule moduleArray[4] =
  {
    {&driveMotor1, &steeringMotor1, &encoder1, {-17.75, 25}},
    {&driveMotor2, &steeringMotor2, &encoder2, {-17.75, -25}},
    {&driveMotor3, &steeringMotor3, &encoder3, {17.75, 25}},
    {&driveMotor4, &steeringMotor4, &encoder4, {17.75, -25}}
  };

  // swerve drive object to control the 4-SwerveModule array using the motion controller object
  SwerveDrive swerve;
  SwervePoseTargeting swerveTargeting{&swerve};

  
  bool isCone = true;
  int as1 = 0; // first (current) arm setpoint
  int as2 = 0; // second (final) arm setpoint
  bool lastButtonState = false;

  ArmController arm{};

  frc::Solenoid yellowLeft{frc::PneumaticsModuleType::REVPH, 14};
  frc::Solenoid yellowRight{frc::PneumaticsModuleType::REVPH, 1};
  frc::Solenoid blueLeft{frc::PneumaticsModuleType::REVPH, 13};
  frc::Solenoid blueRight{frc::PneumaticsModuleType::REVPH, 7};


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
