/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <WPILib.h>
#include <string>
#include <ctre/Phoenix.h>
#include <IterativeRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <MPLeftController.h>
#include <MPRightController.h>
class Robot : public frc::IterativeRobot {
	XboxController Driver;
	XboxController Manipulator;
	WPI_TalonSRX RightDrive;
	WPI_TalonSRX LeftDrive;
	WPI_VictorSPX RightDrive1;
	WPI_VictorSPX RightDrive2;
	WPI_VictorSPX LeftDrive1;
	WPI_VictorSPX LeftDrive2;
	MPLeftController LeftMP;
	MPRightController RightMP;

public:
int Run= 10000;
	Robot(void):
Driver (0),
Manipulator (1),
LeftDrive(3),
	RightDrive (2),
	RightDrive1(6),
	RightDrive2(7),
	LeftDrive1(9),
	LeftDrive2(8),
	LeftMP (LeftDrive),
	RightMP(RightDrive)
{}
	void RobotInit() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;

private:
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};
