/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <frc/WPILib.h>
#include <iostream>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "MPLeftController.h"
#include "MPRightController.h"

void Robot::RobotInit() {

/*
	Good for max speed FPS:.75 and max acceleration: .5 double P=.11; double I=.01;
	Good for max speed FPS:1.5 and max acceleration: 1 double P=.05; double I=.01; double D=.2;
	  */
int Spot = 0;
	double P=.01;
	double I=.01;
	double D=.2;
  
			LeftDrive.Config_kF(0,.14884,10);
			RightDrive.Config_kF(0,.14884,10);
			LeftDrive.EnableCurrentLimit(true);
			LeftDrive.ConfigOpenloopRamp(.125,10);
			LeftDrive.ConfigPeakCurrentLimit(30,10);
			LeftDrive.ConfigPeakCurrentDuration(2000,10);
			LeftDrive.Config_kP(0,P,10);
			LeftDrive.Config_kI(0,I,10);
			LeftDrive.Config_kD(0,D,10);
			LeftDrive.Config_IntegralZone(0,100,10);
			LeftDrive.ConfigSelectedFeedbackSensor(QuadEncoder,0,10);
			LeftDrive.SetSensorPhase(true);
			LeftDrive.ConfigClosedLoopPeakOutput(0,1,10);
			RightDrive.ConfigClosedLoopPeakOutput(0,1,10);
			RightDrive.ConfigClosedloopRamp(.125,10);
			LeftDrive.ConfigClosedloopRamp(.125,10);
			LeftDrive.SetInverted(true);
			LeftDrive.ConfigMotionProfileTrajectoryPeriod(30,10);
			LeftDrive1.SetInverted(LeftDrive.GetInverted());
			LeftDrive2.SetInverted(LeftDrive.GetInverted());
			LeftDrive.ConfigPeakOutputForward(1,10);
			LeftDrive.ConfigPeakOutputReverse(-1,10);
			RightDrive.SetInverted(false);
			RightDrive.ConfigPeakOutputForward(1,10);
			RightDrive.ConfigPeakOutputReverse(-1,10);
			RightDrive1.SetInverted(RightDrive.GetInverted());
			RightDrive2.SetInverted(RightDrive.GetInverted());
			RightDrive.EnableCurrentLimit(true);
			RightDrive.ConfigOpenloopRamp(.125,10);
			RightDrive.ConfigPeakCurrentLimit(30,10);
			RightDrive.ConfigPeakCurrentDuration(2000,10);
			RightDrive.Config_kP(0,P,10);
			RightDrive.Config_kI(0,I,10);
			RightDrive.Config_kD(0,D,10);
			RightDrive.Config_IntegralZone(0,100,10);
			RightDrive.ConfigSelectedFeedbackSensor(QuadEncoder,0,10);
			RightDrive.SetSensorPhase(true);

			RightDrive.ConfigMotionProfileTrajectoryPeriod(30,10);
		//	RightDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic,
			//10, Constants::kTimeoutMs);

  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString(
  //     "Auto Selector", kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
	}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
	}

void Robot::TeleopInit() {}
void Robot::ProfileControl(){
	RightProfile.control();
	RightProfile.PeriodicTask();
	LeftProfile.control();
	LeftProfile.PeriodicTask();
	
	
	if(Robot::Spot>0 and Robot::Spot<4){
	Robot::ProfileRun(Robot::ProfileIdentification);
	SetValueMotionProfile setOutputL = LeftProfile.getSetValue();
		SetValueMotionProfile setOutputR = RightProfile.getSetValue();
	LeftDrive.Set(ControlMode::MotionProfile, setOutputL);
	RightDrive.Set(ControlMode::MotionProfile, setOutputR);
	}
	}
void Robot::ProfileRun(int ProfID){

	switch(Robot::Spot){
	case 0:
	LeftProfile.reset();
	RightProfile.reset();
	Robot::ProfileIdentification = ProfID;
	LeftProfile.ProfileSlotL = ProfID;
	RightProfile.ProfileSlotR = ProfID;
	Robot::Spot = 1;
	break;
	case 1:
	LeftProfile.start();
	RightProfile.start();
	Robot::Spot = 2;
	break;

	}
	}


void Robot::ProfileSchedule(int ProfOne,int ProfTwo, int ProfThree, int ProfFour, int ProfFive, int ProfSix,int ProfSeven){
	switch(Robot::Scheduler){
		case 0:

			if(ProfOne > 0 ){
			Robot::Scheduler++;	
			}else{
			Robot::Scheduler = 1000;
			};
			break;
		case 1:
		Robot::Spot = 0;
		Robot::Scheduler++;
		break;
		case 2:
		
			Robot::ProfileRun(ProfOne);
			if(ProfTwo > 0 and LeftProfile._statusL.isLast==1){
				Robot::Scheduler++;
			}else if(ProfTwo <= 0){
			Robot::Scheduler = 1000;
			}
			break;
			case 3:
		Robot::Spot = 0;
		Robot::Scheduler++;
		break;
		case 4:
	
			Robot::ProfileRun(ProfTwo);
			if(ProfThree > 0 and LeftProfile._statusL.isLast==1){
				Robot::Scheduler++;
			}else if(ProfThree <= 0){
			Robot::Scheduler = 1000;
			}
			break;
			case 5:
		Robot::Spot = 0;
		Robot::Scheduler++;
		break;
		case 6:
		
			Robot::ProfileRun(ProfThree);
			if(ProfFour > 0 and LeftProfile._statusL.isLast==1){
				Robot::Scheduler++;
			}else if(ProfFour <= 0){
			Robot::Scheduler = 1000;
			}
			break;
			case 7:
		Robot::Spot = 0;
		Robot::Scheduler++;
		break;
		case 8:
		
			Robot::ProfileRun(ProfFour);
			if(ProfFive > 0 and LeftProfile._statusL.isLast==1){
				Robot::Scheduler++;
			}else if(ProfFive <= 0){
			Robot::Scheduler = 1000;
			}
			break;
			case 9:
		Robot::Spot = 0;
		Robot::Scheduler++;
		break;
		case 10:
		
			Robot::ProfileRun(ProfFive);
			if(ProfSix > 0 and LeftProfile._statusL.isLast==1){
				Robot::Scheduler++;
			}else if(ProfSix <= 0){
			Robot::Scheduler = 1000;
			}
			break;
			case 11:
		Robot::Spot = 0;
		Robot::Scheduler++;
		break;
		case 12:
		
			Robot::ProfileRun(ProfSix);
			if(ProfSeven > 0 and LeftProfile._statusL.isLast==1){
				Robot::Scheduler++;
			}else if(ProfSeven <= 0){
			Robot::Scheduler = 1000;
			}
			break;
			case 13:
		Robot::Spot = 0;
		Robot::Scheduler++;
		break;
		case 14:
		
			Robot::ProfileRun(ProfSeven);
			if(LeftProfile._statusL.isLast==1){
				Robot::Scheduler = 1000;
			}
			break;
			case 15:
		Robot::Spot = 10;
		Robot::Scheduler++;
		break;
		case 1000:
		
		break;
			}
	}
void Robot::TeleopPeriodic() {
	ProfileControl();
	/*if(Driver.GetYButtonPressed()==1){
		
		Robot::Spot = 0;
		Robot::ProfileRun(0);
		}
	if(Driver.GetBButtonPressed()==1){
			
			Robot::Spot = 0;
			Robot::ProfileRun(1);
		}
	*/
/*if(Driver.GetAButtonPressed()==1){
		Robot::Spot = 0;
			Robot::ProfileRun(6);
	}*/
	/*
	if(Driver.GetXButtonPressed()==1){
		Robot::Spot = 0;
		Robot::ProfileRun(3);
	}
	if(Driver.GetBumperPressed(frc::XboxController::kLeftHand)==1){
		Robot::Spot = 0;
		Robot::ProfileRun(4);
	}
	if(Driver.GetBumperPressed(frc::XboxController::kRightHand)==1){
		Robot::Spot = 0;
		Robot::ProfileRun(5);
	}*/
	/*if(Driver.GetAButtonPressed()==1){
			Robot::Scheduler =0;
		}*/
	//Robot::ProfileSchedule(1,2,3,4,5,0,0);
	LeftDrive.Set(ControlMode::PercentOutput,Driver.GetY(frc::XboxController::kLeftHand));
	RightDrive.Set(ControlMode::PercentOutput,Driver.GetY(frc::XboxController::kRightHand));
		frc::SmartDashboard::PutNumber("SPot",Spot);
		frc::SmartDashboard::PutNumber("PRofileID",Robot::ProfileIdentification);
		frc::SmartDashboard::PutNumber("VelocityR",RightDrive.GetSelectedSensorVelocity(0));
		frc::SmartDashboard::PutNumber("VelocityL",LeftDrive.GetSelectedSensorVelocity(0));
		frc::SmartDashboard::PutNumber("RotR",RightDrive.GetSelectedSensorPosition(0)/4096);
		frc::SmartDashboard::PutNumber("RotL",LeftDrive.GetSelectedSensorPosition(0)/4096);
		frc::SmartDashboard::PutNumber("Scheduler",Robot::Scheduler);
		frc::SmartDashboard::PutBoolean("isLast",LeftProfile._statusL.isLast);
		frc::SmartDashboard::PutNumber("bottombufferL",LeftProfile._statusL.btmBufferCnt);
		frc::SmartDashboard::PutNumber("topbufferL",LeftProfile._statusL.topBufferCnt);
		frc::SmartDashboard::PutNumber("bottombufferR",RightProfile._statusR.btmBufferCnt);
		frc::SmartDashboard::PutNumber("topbufferR",RightProfile._statusR.topBufferCnt);

	


	
  


	LeftDrive1.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
	LeftDrive2.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
	RightDrive1.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());
	RightDrive2.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());

	}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
