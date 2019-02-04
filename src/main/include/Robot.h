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

#include <MPRight.h>
class Robot : public frc::IterativeRobot {
	XboxController Driver;
	XboxController Manipulator;
	WPI_TalonSRX RightDrive;
	WPI_TalonSRX Lift;
	WPI_TalonSRX LeftDrive;
	WPI_VictorSPX RightDrive1;
	WPI_VictorSPX RightDrive2;
	WPI_VictorSPX LeftDrive1;
	WPI_VictorSPX LeftDrive2;
	//MPL LeftMP;
	//MP RightMP;
	PigeonIMU Pigeon;
	BufferedTrajectoryPointStream LeftMP;
	BufferedTrajectoryPointStream RightMP;
	
 

public:
int total;
int spot = 100;
int pop = 100;
bool drive = 0;
	Robot(void):
	Lift (1),
Driver (0),
Manipulator (1),
LeftDrive(3),
	RightDrive (2),
	RightDrive1(6),
	RightDrive2(7),
	LeftDrive1(9),
	LeftDrive2(8),
	//LeftMP (LeftDrive),
	//RightMP(RightDrive),
	Pigeon(&Lift)
{}
	void fillBuffer(int Sz,double Prof[][3]);
	void executeProfile();
	void FollowPath();
	void RunProfile(int ProfID);
	void ProfileControl();
	void RobotInit() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;
  void RobotPeriodic() override;
private:
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};
