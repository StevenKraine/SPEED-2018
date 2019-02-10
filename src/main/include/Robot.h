/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <ctre/Phoenix.h>
#include <string>
#include <WPILib.h>
#include <frc/IterativeRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <PlotThread.h>
#include "MasterProfileConfiguration.h"
#include "FollowerProfileConfiguration.h"

class Robot : public frc::IterativeRobot {

 frc::XboxController Driver;
	frc::XboxController Manipulator;
	WPI_TalonSRX RightDrive;
	WPI_TalonSRX Lift;
	WPI_TalonSRX LeftDrive;
	WPI_VictorSPX RightDrive1;
	WPI_VictorSPX RightDrive2;
	WPI_VictorSPX LeftDrive1;
	WPI_VictorSPX LeftDrive2;
	PigeonIMU Pigeon;
	BufferedTrajectoryPointStream Profile;
	BufferedTrajectoryPointStream Profile1;
	BufferedTrajectoryPointStream FastBack;
	BufferedTrajectoryPointStream Turnback;
	//BufferedTrajectoryPointStream
//	BufferedTrajectoryPointStream
//	BufferedTrajectoryPointStream
//	BufferedTrajectoryPointStream
//	BufferedTrajectoryPointStream
//	BufferedTrajectoryPointStream
//	BufferedTrajectoryPointStream
//	BufferedTrajectoryPointStream
//	BufferedTrajectoryPointStream
//	BufferedTrajectoryPointStream
///	BufferedTrajectoryPointStream
//	BufferedTrajectoryPointStream
//	BufferedTrajectoryPointStream

	FollowerProfileConfiguration Follow;
  MasterProfileConfiguration Master;
	PlotThread Plot;
	frc::Joystick Manipulatortwo;
	//frc::Compressor pressor;
 public:
 bool ButtonOne;
 bool ButtonTwo;
 bool ButtonThree;
 bool ButtonFour;
 bool ButtonFive;
 bool ButtonSix;
 bool ButtonSeven;
 bool ButtonEight;
 bool ButtonNine;
 bool ButtonTen;
 bool ButtonEleven;
 bool ButtonTwelve;
 bool ButtonThirteen;
 bool ButtonFourteen;
 int count;
 
 int button;
int _state;
int total;
int spot = 100;
int pop = 100;
bool drive = 0;
int Profidilly;
	Robot(void):
	Manipulatortwo (2),
	Plot (&LeftDrive),
  Follow(),
	Lift (1),
Driver (0),
Manipulator (1),
LeftDrive(3),
	RightDrive (2),
	RightDrive1(6),
	RightDrive2(7),
	LeftDrive1(9),
	LeftDrive2(8),
	Pigeon(&Lift),
  Master(&RightDrive,&Pigeon)
 
 
 {};
 void RunProfile(BufferedTrajectoryPointStream& Buffer, const double profile[][4], int totalCnt,int direction);
 void InitBuffer(BufferedTrajectoryPointStream& ProfID, const double profile[][4], int totalCnt, int direction);
 void InitBuffer1(const double profile[][4], int totalCnt, int direction);
 void LiftControl();
 void IntakeControl();
 void Limelight();
 void SmartDashboard();
 void Followers();

  void RobotInit() override;
  void RobotPeriodic() override;
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
