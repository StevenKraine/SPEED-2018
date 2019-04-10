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
#include <PlotThread.h>
#include "MasterProfileConfiguration.h"
#include "FollowerProfileConfiguration.h"
#include "math.h"

class Robot : public frc::IterativeRobot {

 frc::XboxController Driver;
	frc::XboxController Manipulator;
	WPI_TalonSRX RightDrive;
	WPI_TalonSRX Lift;
	WPI_VictorSPX LiftFollower;
	WPI_TalonSRX LeftDrive;
	WPI_TalonSRX IntakeArm;
	WPI_TalonSRX Intake;
	WPI_VictorSPX ClimbFoot;
	WPI_VictorSPX Climb2;
	WPI_VictorSPX RightDrive1;
	WPI_VictorSPX RightDrive2;
	WPI_VictorSPX LeftDrive1;
	WPI_VictorSPX LeftDrive2;
	//WPI_VictorSPX Stabby;
	WPI_VictorSPX RollerClaw;
	WPI_VictorSPX Vacuum;
	WPI_TalonSRX Climb1;
	PigeonIMU Pigeon;
	BufferedTrajectoryPointStream Profile;
	BufferedTrajectoryPointStream Profile1;
	BufferedTrajectoryPointStream FastBack;
	BufferedTrajectoryPointStream Turnback;
	BufferedTrajectoryPointStream CargoToFeederClose;
	BufferedTrajectoryPointStream TurnAroundFar;
	BufferedTrajectoryPointStream FastStraight;
	BufferedTrajectoryPointStream TurnAroundToFeeder;
	BufferedTrajectoryPointStream LeftPlattoLeftRocketNearA;
	BufferedTrajectoryPointStream CargoShortTurnRight;
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
	frc::Compressor pressor;
	frc::Solenoid StabbyS;
	frc::Solenoid StabbyClaw;
	frc::DoubleSolenoid ClimbBase;
 public:
 
 bool stabbyin;
 int ElevatorHeight;
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
 int run;
 int button;
int _state;
int total;
bool ClimbOveride;
int spot = 100;
int pop = 100;
bool drive = 0;
int Profidilly;
//int HatchIntake = 500;
double LimelightHieght = 37.5;
double LimelightAngleVertical = 20;
double LimelightAngleHorizontal = 0;
double HatchHieght = 31.5;
double BallHieght = 39.125; //ish
double HDistance;
double BDistance;
double offset;
double HHieght = (LimelightHieght-HatchHieght);
double BHieght =(LimelightHieght-BallHieght);
bool Reverse = 1;
bool Item = 0;
bool finger;
int Limelightplace;
int loop;
int loopB;
double XValue;

double SetAngle;
int loopA;
int loopidee;

#define PI 3.1415926535897
	Robot(void):
	pressor(0),
	Manipulatortwo (2),
	Follow(),
	Plot (&LeftDrive),
	Driver (0),
	Manipulator (1),
	LiftFollower(5),
	Lift (5),	
	LeftDrive(3),
	Intake(1),
	RightDrive (2),
	RightDrive1(3),
	RightDrive2(4),
	LeftDrive1(1),
	LeftDrive2(2),
	Pigeon(&Intake),
	Vacuum(7),
	RollerClaw(6),
	IntakeArm(4),
	Climb1 (8),
	Climb2 (9),
	StabbyS (2),
	StabbyClaw (3),
	ClimbBase(0,1),
	ClimbFoot(8),
	
 	Master(&RightDrive,&Pigeon)
 
 
 {}
 void ClimbControl();
 void ArcadeDrive(bool Go);
 void RollerClawControl();
 void StabbyControl();
 void RunProfile(BufferedTrajectoryPointStream& Buffer);
 void InitBuffer(BufferedTrajectoryPointStream& ProfID, const double profile[][4], int totalCnt, int direction);
 void InitBuffer1(const double profile[][4], int totalCnt, int direction);
 void LiftControl();
 void IntakeControl();
 void Limelight();
 void SmartDashboard();
 void Followers();
 void JoystickDrive();
 void JoystickDriveOne(bool Go);
 void AutonLimelight(bool Go);
 void ElevatorSet(int Position);
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Driver Control";
  const std::string kAutoNameCustom = "My Auto";
  const std::string kAutoNameAUTOTHREE = "Auto 3";
  std::string m_autoSelected;
};
