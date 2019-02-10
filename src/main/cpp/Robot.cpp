/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "Instrum.h"
#include <iostream>
#include "Profile.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTable.h>
#include "networktables/NetworkTableInstance.h"

void Robot::RobotInit() {
  //InitBuffer(Profile, kStraightCenter, kStraightCenterSz,1);
  InitBuffer(Profile1, kCargoToFeeder,kCargoToFeederSz,-1);
    InitBuffer(Turnback, kTurnAround, kTurnAroundSz,-1);
 // InitBuffer(FastBack, kStraightCenter, kStraightCenterSz,1);
Profidilly = 0;
  _state = 0;
  LeftDrive.ClearMotionProfileTrajectories();
  RightDrive.ClearMotionProfileTrajectories();
  RightDrive.ConfigAllSettings(Follow);
  LeftDrive.ConfigAllSettings(Master);

  LeftDrive.SetSensorPhase(true);
      RightDrive.SetSensorPhase(true);

      LeftDrive.SetInverted(true);
      RightDrive.SetInverted(false);

      LeftDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 20); //Telemetry using Phoenix Tuner

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
void Robot::SmartDashboard(){
  frc::SmartDashboard::PutNumber("Profidilly", 	Profidilly);
  frc::SmartDashboard::PutNumber("state", 	_state);
	frc::SmartDashboard::PutNumber("Buffer", 	LeftDrive.GetMotionProfileTopLevelBufferCount());
  frc::SmartDashboard::PutBoolean("1",ButtonOne);
  frc::SmartDashboard::PutBoolean("2",ButtonTwo);
  frc::SmartDashboard::PutBoolean("3",ButtonThree);
  frc::SmartDashboard::PutBoolean("4",ButtonFour);
  frc::SmartDashboard::PutBoolean("5",ButtonFive);
  frc::SmartDashboard::PutBoolean("6",ButtonSix);
  frc::SmartDashboard::PutBoolean("7",ButtonSeven);
  frc::SmartDashboard::PutBoolean("8",ButtonEight);
  frc::SmartDashboard::PutBoolean("9",ButtonNine);
  frc::SmartDashboard::PutBoolean("10",ButtonTen);
  frc::SmartDashboard::PutBoolean("11",ButtonEleven);
  frc::SmartDashboard::PutBoolean("12",ButtonTwelve);
  frc::SmartDashboard::PutBoolean("13",ButtonThirteen);
  frc::SmartDashboard::PutBoolean("14",ButtonFourteen);
  ButtonOne = Manipulatortwo.GetRawButton(1);
   ButtonTwo = Manipulatortwo.GetRawButton(2);
   ButtonThree= Manipulatortwo.GetRawButton(3);
   ButtonFour =Manipulatortwo.GetRawButton(4);
   ButtonFive =Manipulatortwo.GetRawButton(5);
   ButtonSix =Manipulatortwo.GetRawButton(6);
   ButtonSeven= Manipulatortwo.GetRawButton(7);
   ButtonEight =Manipulatortwo.GetRawButton(8);
   ButtonNine =Manipulatortwo.GetRawButton(9);
   ButtonTen =Manipulatortwo.GetRawButton(10);
   ButtonEleven= Manipulatortwo.GetRawButton(11);
   ButtonTwelve =Manipulatortwo.GetRawButton(12);
   ButtonThirteen= Manipulatortwo.GetRawButton(13);
   ButtonFourteen =Manipulatortwo.GetRawButton(14);
  }
void Robot::Followers(){ LeftDrive1.Follow(LeftDrive, FollowerType_PercentOutput);
	LeftDrive2.Follow(LeftDrive, FollowerType_PercentOutput);
	RightDrive1.Follow(RightDrive, FollowerType_PercentOutput);
	RightDrive2.Follow(RightDrive, FollowerType_PercentOutput);
  RightDrive1.SetInverted(RightDrive.GetInverted());
			RightDrive2.SetInverted(RightDrive.GetInverted());
      LeftDrive1.SetInverted(LeftDrive.GetInverted());
			LeftDrive2.SetInverted(LeftDrive.GetInverted());}
void Robot::RunProfile(BufferedTrajectoryPointStream& Buffer,const double profile[][4], int totalCnt,int direction){
    InitBuffer(Buffer,profile,totalCnt,direction);
    LeftDrive.ClearMotionProfileTrajectories();
    LeftDrive.GetSensorCollection().SetQuadraturePosition(0);
            RightDrive.GetSensorCollection().SetQuadraturePosition(0);
            Pigeon.SetYaw(0);
             RightDrive.Follow(LeftDrive, FollowerType_AuxOutput1);
            LeftDrive.StartMotionProfile(Buffer, 120, ControlMode::MotionProfileArc);
 
  }
void Robot::TeleopPeriodic() {
  Robot::Followers();
  Robot::SmartDashboard();
 // Robot::Limelight();
 // Robot::LiftControl();
 // Robot::IntakeControl();
 
    double ypr [3];
   Pigeon.GetYawPitchRoll(ypr);
     frc::SmartDashboard::PutNumber("yaw", ypr[0]);
if(Driver.GetAButton()==1 or Driver.GetBButton()==1 or Driver.GetXButton()==1 or Driver.GetYButton()==1){
count = 100;
}
if(count <10){
  LeftDrive.Set(ControlMode::PercentOutput,Driver.GetY(frc::XboxController::kLeftHand));
   RightDrive.Set(ControlMode::PercentOutput,Driver.GetY(frc::XboxController::kRightHand));
}
  if(Driver.GetAButtonPressed()==1){ 
       LeftDrive.ClearMotionProfileTrajectories();

    Profidilly = 0;
      Robot::RunProfile(FastBack,kStraightCenter,kStraightCenterSz,1);
}
 if(Driver.GetBButtonPressed()==1){ 
   LeftDrive.ClearMotionProfileTrajectories();
    Profidilly = 0;
      Robot::RunProfile(FastBack,kCargoToFeeder,kCargoToFeederSz,-1);
}


    switch(Profidilly){
      case 0:
if(LeftDrive.IsMotionProfileFinished()==1){
  Profidilly = 1;
}
      break;
      case 1:

      break;
    }
   
	

  }

void Robot::TestPeriodic() {

  }
void Robot::IntakeControl(){

  }
void Robot::LiftControl(){
  if(Manipulatortwo.GetRawButton(6)==1){
  Lift.Set(ControlMode::Position,6000);
  }
  if(Manipulatortwo.GetRawButton(7)==1){
    Lift.Set(ControlMode::Position,4000);
  }
  if(Manipulatortwo.GetRawButton(8)==1){
    Lift.Set(ControlMode::Position,2000);
  }
  if(Manipulatortwo.GetRawButton(5)==1){
    Lift.Set(ControlMode::Position,0);
  }



  }
void Robot::Limelight(){
	   std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
  double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
  double targetArea = table->GetNumber("ta",0.0);
  double targetSkew = table->GetNumber("ts",0.0);
  }

void Robot::InitBuffer(BufferedTrajectoryPointStream& ProfID,const double profile[][4], int totalCnt,int direction)
  {
      bool forward = true; // set to false to drive in opposite direction of profile (not really needed
                          // since you can use negative numbers in profile).

      TrajectoryPoint point; // temp for for loop, since unused params are initialized
                            // automatically, you can alloc just one

      /* clear the buffer, in case it was used elsewhere */
      ProfID.Clear();

      //double turnAmount = rotations * 8192.0; //8192 units per rotation for a pigeon


      /* Insert every point into buffer, no limit on size */
      for (int i = 0; i < totalCnt; ++i) {

          double Direction = direction;//d forward ? +1 : -1;
          double positionRot = profile[i][0];
          double velocityRPM = profile[i][1];
          double Angle;
          if(direction<0){
            Angle = profile[i][3] -3.1415962;
          }else{
              Angle = profile[i][3] ;
          }
        
          int durationMilliseconds = (int) profile[i][2];

          /* for each point, fill our structure and pass it to API */
          point.timeDur = durationMilliseconds;
          point.position = Direction * positionRot /*(1/.6366)*/* 5190 ;//(8192/3.1415962); // Convert Revolutions to
                                                          // Units
          point.velocity = Direction * velocityRPM * (1/.6366)/ 600.0 ;//(8192/3.1415962); // Convert RPM to
                                                                  // Units/100ms
          
          /** 
           * Here is where you specify the heading of the robot at each point. 
           * In this example we're linearly interpolating creating a segment of a circle to follow
           */
          point.auxiliaryPos = Angle *(8192/360)*(180/3.14159)+40; //turnAmount * ((double)i / (double)totalCnt); //Linearly interpolate the turn amount to do a circle
          point.auxiliaryVel = 0;


          point.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
          point.profileSlotSelect1 = 1; /* which set of gains would you like to use [0,3]? */
          point.zeroPos = (i == 0); /* set this to true on the first point */
          point.isLastPoint = ((i + 1) == totalCnt); /* set this to true on the last point */
          point.arbFeedFwd = 0; /* you can add a constant offset to add to PID[0] output here */

          point.useAuxPID = true; /* Using auxiliary PID */
          ProfID.Write(point);
  }
  }
void Robot::InitBuffer1(const double profile[][4], int totalCnt,int direction)
  {
      bool forward = true; // set to false to drive in opposite direction of profile (not really needed
                          // since you can use negative numbers in profile).

      TrajectoryPoint point; // temp for for loop, since unused params are initialized
                            // automatically, you can alloc just one

      /* clear the buffer, in case it was used elsewhere */
      Profile1.Clear();

      //double turnAmount = rotations * 8192.0; //8192 units per rotation for a pigeon


      /* Insert every point into buffer, no limit on size */
      for (int i = 0; i < totalCnt; ++i) {

          double Direction = direction;//d forward ? +1 : -1;
          double positionRot = profile[i][0];
          double velocityRPM = profile[i][1];
          double Angle;
          if(direction<0){
            Angle = profile[i][3] -3.1415962;
          }else{
              Angle = profile[i][3] ;
          }
        
          int durationMilliseconds = (int) profile[i][2];

          /* for each point, fill our structure and pass it to API */
          point.timeDur = durationMilliseconds;
          point.position = Direction * positionRot /*(1/.6366)*/* 5190 ;//(8192/3.1415962); // Convert Revolutions to
                                                          // Units
          point.velocity = Direction * velocityRPM * (1/.6366)/ 600.0 ;//(8192/3.1415962); // Convert RPM to
                                                                  // Units/100ms
          
          /** 
           * Here is where you specify the heading of the robot at each point. 
           * In this example we're linearly interpolating creating a segment of a circle to follow
           */
          point.auxiliaryPos = Angle *(8192/360)*(180/3.14159)+40; //turnAmount * ((double)i / (double)totalCnt); //Linearly interpolate the turn amount to do a circle
          point.auxiliaryVel = 0;


          point.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
          point.profileSlotSelect1 = 1; /* which set of gains would you like to use [0,3]? */
          point.zeroPos = (i == 0); /* set this to true on the first point */
          point.isLastPoint = ((i + 1) == totalCnt); /* set this to true on the last point */
          point.arbFeedFwd = 0; /* you can add a constant offset to add to PID[0] output here */

          point.useAuxPID = true; /* Using auxiliary PID */
          Profile1.Write(point);
      }
    }

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
