/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <ctre/Phoenix.h>
#include <string>
#include <frc/WPILib.h>
#include <frc/IterativeRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <MPRightController.h>
#include <MPLeftController.h>


class Robot : public frc::IterativeRobot {
frc::XboxController Driver;
frc::XboxController Manipulator;
WPI_TalonSRX LeftDrive;
WPI_TalonSRX RightDrive;
WPI_VictorSPX LeftDrive1;
WPI_VictorSPX LeftDrive2;
WPI_VictorSPX RightDrive1;
WPI_VictorSPX RightDrive2;
MPLeft LeftProfile;
MPRight RightProfile;

 public:
 int Scheduler = 100;
 int ProfileSlot = 0;
 int Spot = 1000;
 int ProfileIdentification = -1;
 
 Robot():
 Driver (0),
Manipulator (1),
LeftDrive(3),
	RightDrive (2),
	RightDrive1(6),
	RightDrive2(7),
	LeftDrive1(9),
	LeftDrive2(8),
 LeftProfile (LeftDrive),
 RightProfile (RightDrive)
 
 {}
 
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void ProfileRun(int ProfId);
  void ProfileControl();
  void ProfileSchedule(int ProfOne,int ProfTwo, int ProfThree, int ProfFour, int ProfFive, int ProfSix,int ProfSeven);
 private:
  frc::SendableChooser<std::string> m_chooser;
  frc::SendableChooser<std::string> ChoiceOne;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
   const std::string One = "One";
  const std::string Two = "Two";
  
  std::string m_autoSelected;
  std::string Choice;
};
