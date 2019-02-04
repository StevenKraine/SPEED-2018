/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
//#include <pathfinder.h>
#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
#include <WPILib.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "LeftProfile.h"
void Robot::executeProfile()
{
    LeftDrive.StartMotionProfile(LeftMP, 10, ControlMode::MotionProfile);
}
void Robot::RobotInit() {
  double P=0.1;//.05
	double I=0.0;//.01
	double D=0.0;//.2

			LeftDrive.Config_kF(0,.5,10);
			RightDrive.Config_kF(0,.5,10);
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
void Robot::fillBuffer(int Sz, double Prof[][3])
{
    TrajectoryPoint pointL; //Create just one trajectorypoint to write with
    for(int i = 0; i < Sz; i++)
    {
        pointL.useAuxPID = false; //We aren't using aux pid
        pointL.profileSlotSelect0 = 0; //Use slot 0 for PIDF Gains

        if(i == 0) pointL.zeroPos = true;
        else pointL.zeroPos = false;

        pointL.position = Prof[i][0]; //Scale to talon native units
        pointL.velocity = Prof[i][1]; //Scale to talon native units
        pointL.timeDur = Prof[i][2]; //Milliseconds
        
        if(i == Sz - 1) pointL.isLastPoint = true;
        else pointL.isLastPoint = false;

        LeftMP.Write(pointL);


    }
}
void Robot::TeleopInit() {}
void Robot::ProfileControl(){
 
 /*   SetValueMotionProfile setOutputL = LeftMP.getSetValue();
		SetValueMotionProfile setOutputR = RightMP.getSetValue();
		
	LeftDrive.Set(ControlMode::MotionProfile, setOutputL);
	RightDrive.Set(ControlMode::MotionProfile, setOutputR);
  RightMP.control();
	RightMP.PeriodicTask();
	LeftMP.control();
	LeftMP.PeriodicTask();*/
 
 	if(Robot::spot<=2 and Robot::spot>0){
     Robot::RunProfile(Robot::pop);
  }
	}
void Robot::RunProfile(int ProfID){
 
 /* switch(Robot::spot){
  case 0:
  LeftMP.reset();
  RightMP.reset();
  LeftMP.ProfileID = ProfID;
  RightMP.ProfileID = ProfID;
   Robot::spot = 1;
	 
  Robot::pop = ProfID;
  
 
  break;
  case 1:
  LeftMP.start();
  RightMP.start();

  Robot::spot = 2;
  break;
  case 2:
 

  

  
  break;
  
  }*/
	}
void Robot::FollowPath(){
 for(int i = 0; i < Robot::total; i++){

 }
	}
void Robot::TeleopPeriodic() {
	//	LeftDrive.StartMotionProfile()

	std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
	double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
	double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
	double targetArea = table->GetNumber("ta",0.0);
	double targetSkew = table->GetNumber("ts",0.0);

	
	if(Driver.GetBButtonPressed()==1){
		Robot::drive = !Robot::drive;
	}
	if(Driver.GetYButtonPressed()==1){
  fillBuffer(RocketszL,RocketL);
  // Robot::spot = 0;
  // Robot::RunProfile(1);
	};
	 if(Driver.GetXButtonPressed()==1){
  executeProfile();
  // Robot::spot = 0;
   //Robot::RunProfile(4);
	};
	/*if(Robot::drive == 1){
		LeftDrive.Set(ControlMode::PercentOutput,Driver.GetY(frc::XboxController::kLeftHand));
		RightDrive.Set(ControlMode::PercentOutput,Driver.GetY(frc::XboxController::kRightHand));
	}else{Robot::ProfileControl();
 
  if(Driver.GetAButtonPressed()==1){
	Robot::spot = 0;
   Robot::RunProfile(2);
	 /*	if(LeftMP._statusL.btmBufferCnt>100 and RightMP._statusR.btmBufferCnt >100){
		 LeftDrive.ConfigPeakOutputForward(0,10);
		 LeftDrive.ConfigPeakOutputReverse(0,10);
		 RightDrive.ConfigPeakOutputForward(0,10);
		 RightDrive.ConfigPeakOutputReverse(0,10);
	 }else{
		 LeftDrive.ConfigPeakOutputForward(1,10);
		 LeftDrive.ConfigPeakOutputReverse(-1,10);
		 RightDrive.ConfigPeakOutputForward(1,10);
		 RightDrive.ConfigPeakOutputReverse(-1,10);
	 }
  }*/

  //}
 LeftDrive.ConfigPeakOutputForward(1,10);
		 LeftDrive.ConfigPeakOutputReverse(-1,10);
		 RightDrive.ConfigPeakOutputForward(1,10);
		 RightDrive.ConfigPeakOutputReverse(-1,10);

	Pigeon.GetFusedHeading();
	double xyz[3];
	Pigeon.GetAccumGyro(xyz);

 int LeftEn = LeftDrive.GetSelectedSensorPosition(0);
 int RightEn = RightDrive.GetSelectedSensorPosition(0);
 	LeftDrive1.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
	LeftDrive2.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
	RightDrive1.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());
	RightDrive2.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());
	LeftDrive.GetMotionProfileTopLevelBufferCount();
  frc::SmartDashboard::PutNumber("spot",Robot::spot);
 /* frc::SmartDashboard::PutNumber("LeftBtm",LeftMP._statusL.btmBufferCnt);
	 frc::SmartDashboard::PutNumber("RightBtm",RightMP._statusR.btmBufferCnt);
	if(Driver.GetYButtonPressed()==1){

	Pigeon.SetAccumZAngle(0,10);

	Robot::spot = 0;
	}
	frc::SmartDashboard::PutNumber("horizontal",targetOffsetAngle_Horizontal);*/
		frc::SmartDashboard::PutNumber("rightEncoder", RightEn/4096);
	frc::SmartDashboard::PutNumber("LeftEncoder", LeftEn/4096);
	frc::SmartDashboard::PutNumber("Buffer", 	LeftDrive.GetMotionProfileTopLevelBufferCount());
	frc::SmartDashboard::PutNumber("x",xyz[1]);
	frc::SmartDashboard::PutNumber("y",xyz[2]);
	frc::SmartDashboard::PutNumber("z",xyz[3]);


	}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
