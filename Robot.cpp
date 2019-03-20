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
#include "math.h"
void Robot::RobotInit() {

   frc::CameraServer::GetInstance()->StartAutomaticCapture();
  stabbyin = 0;
  pressor.SetClosedLoopControl(true);
  InitBuffer(Profile, kStraightCenter, kStraightCenterSz,1);
  InitBuffer(Profile1, kCargoToFeeder,kCargoToFeederSz,-1);
  InitBuffer(Turnback,kTurnAround,kTurnAroundSz,-1);
  InitBuffer(CargoToFeederClose,kcargotofeedernorm,kcargortofeedernormsz,1);
  InitBuffer(TurnAroundFar,kTurnAroundFar,kTurnAroundFarSZ,-1);
  InitBuffer(FastStraight, kStraight, kStraightSz,-1);
  InitBuffer(TurnAroundToFeeder, kturnaroundtofeeder, kturnaroundtofeederSz,1);
  InitBuffer( LeftPlattoLeftRocketNearA, kLeftPlattoLeftRocketNear, kLeftPlattoLeftRocketNearSz,1);
  InitBuffer(FastBack, kstraightstraight, kstraightstraightSz,-1);
  InitBuffer(CargoShortTurnRight, kcargoshortturnright, kcargoshortturnrightsz,-1);
  Profidilly = 0;
  _state = 0;
  LeftDrive.ClearMotionProfileTrajectories();
  RightDrive.ClearMotionProfileTrajectories();
  RightDrive.ConfigAllSettings(Follow);
  LeftDrive.ConfigAllSettings(Master);
  Lift.ConfigSelectedFeedbackSensor(QuadEncoder,0,10);
  Lift.SetSensorPhase(true);
  Lift.SetInverted(false);
  Lift.Config_kP(0,.2,10);
  Lift.Config_kI(0,.0,10);
  Lift.Config_kD(0,0,10);
  Climb1.SetInverted(true);
  Climb2.SetInverted(true);
  Climb1.ConfigPeakOutputForward(1,10);
  Climb1.ConfigPeakOutputReverse(-1,10);
  LiftFollower.SetInverted(!Lift.GetInverted());
  LiftFollower.Follow(Lift,FollowerType::FollowerType_PercentOutput);
  LeftDrive.SetSensorPhase(true);
  RightDrive.SetSensorPhase(true);
  LeftDrive.SetInverted(true);
  RightDrive.SetInverted(false);
  LeftDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 20); //Telemetry using Phoenix Tuner
  double P=0.3;
  double I=0;
  double D=0;
  double F=0;
  IntakeArm.Config_kP(0,P,10);
  IntakeArm.Config_kI(0,I,10);
  IntakeArm.Config_kD(0,10);
  IntakeArm.Config_kF(0,0,10);
  IntakeArm.ConfigSelectedFeedbackSensor(QuadEncoder,0,10);
  IntakeArm.EnableCurrentLimit(true);
  IntakeArm.SetSensorPhase(false);
  IntakeArm.SetInverted(true);
  IntakeArm.ConfigOpenloopRamp(0,10);
  IntakeArm.ConfigClosedloopRamp(.125,10);
  IntakeArm.ConfigPeakCurrentLimit(20,10);
  Lift.ConfigClosedloopRamp(.125,10);
  Lift.ConfigPeakOutputForward(1,10);
  Lift.ConfigPeakOutputReverse(-.6,10);
  IntakeArm.ConfigPeakOutputForward(1,10);
  IntakeArm.ConfigPeakOutputReverse(-1,10);
  IntakeArm.ConfigPeakCurrentDuration(0,10);
  IntakeArm.SetNeutralMode(NeutralMode::Brake);
  run = 0;
  ClimbOveride = 0;
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
     m_chooser.AddOption(kAutoNameAUTOTHREE, kAutoNameAUTOTHREE);
     
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

  }else{

  }
  }

void Robot::AutonomousPeriodic() {
  double ypr [3];
    Pigeon.GetYawPitchRoll(ypr);
  	     double Goal = 90;
  double offsetTurnL = Goal - ypr[0];
   double offsetTurnR = Goal - ypr[0];
          Robot::StabbyControl();  
  Robot::LiftControl();
  Robot::Followers();
  Robot::SmartDashboard();
  Robot::IntakeControl();
  Robot::RollerClawControl();
if (m_autoSelected == kAutoNameCustom) {
    loopA++;
    frc::SmartDashboard::PutNumber("LoopA",loopA);
       std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
      double targetArea = table->GetNumber("ta",0.0); 
   double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
      switch(run){
       case 0:
       run++;
       break;

  case 1:
   nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline",1);

 AutonLimelight(1);
 JoystickDriveOne(0);
 
 if(targetArea>2.75 or Driver.GetAButtonPressed()==1){
   loopA = 0;
   run++;
   AutonLimelight(0);
 }
 break;
 case 2:
 JoystickDriveOne(1);
 if(Driver.GetAButtonPressed()==1){
   loopA = 0;
    JoystickDriveOne(0);
   run++;
 }
 break;
 
 
 case 3:
 JoystickDriveOne(0);
 AutonLimelight(0);
 JoystickDrive();

 if(Driver.GetAButtonPressed()==1 and loopA>10){
   loopA = 0;
   run++;

 }
 break;
 case 4:
   AutonLimelight(1);
   JoystickDriveOne(0);
 if((targetArea>2.75 or Driver.GetAButtonPressed()==1) and loopA>10){
   loopA = 0;
   run++;
   AutonLimelight(0);
 }
   break;
   case 5:
   JoystickDriveOne(1);
 if(Driver.GetAButtonPressed()==1){
   loopA = 0;
    JoystickDriveOne(0);
   run++;
 }
   break;
   case 6:

 JoystickDrive();
 JoystickDriveOne(0);
  AutonLimelight(0);
 if(Driver.GetAButtonPressed()==1){
   loopA = 0;
   run++;

 }
 break;
 case 7:
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline",0);

   AutonLimelight(1);
   JoystickDriveOne(0);
 if(targetArea>2.75 or Driver.GetAButtonPressed()==1){
   loopA = 0;
   run++;
   AutonLimelight(0);
 }
   break;
   case 8:
   JoystickDriveOne(1);
 if(Driver.GetAButtonPressed()==1){
   loopA = 0;
    JoystickDriveOne(0);
   run++;
 }
 break;
 case 9:
  JoystickDrive();
 break;
   break;
      }
  }
else if(m_autoSelected == kAutoNameAUTOTHREE){
 loopA++;
    frc::SmartDashboard::PutNumber("LoopA",loopA);
       std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
      double targetArea = table->GetNumber("ta",0.0); 
   double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
      switch(run){
       case 0:
  Pigeon.SetYaw(0);
  run++;
  break;

   case 1:
             nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline",0);

 
 AutonLimelight(1);
 JoystickDriveOne(0);
 
 if(targetArea>2.75 or Driver.GetAButtonPressed()==1){
   loopA = 0;
   run++;
   AutonLimelight(0);
   
 }
 break;
 case 2:
 JoystickDriveOne(1);
 if(Driver.GetAButtonPressed()==1){
   loopA = 0;
    JoystickDriveOne(0);
   run++;
  
 }
 break;
 
 
 case 3:
 
  LeftDrive.Set(ControlMode::PercentOutput,.0055*offsetTurnL);
    RightDrive.Set(ControlMode::PercentOutput,-.0055*offsetTurnR);
 if(ypr[0]>88 or Driver.GetAButtonPressed()==1){
   loopA = 0;
   run++;

 }
 break;
 case 4:
  Profidilly= 0;
  RunProfile(CargoShortTurnRight);

  run=30;

 break;
 case 30:
  if(LeftDrive.IsMotionProfileFinished()==1){
    run=5;
  }
 break;
 case 5:
   AutonLimelight(1);
   JoystickDriveOne(0);
 if(targetArea>2.75 or Driver.GetAButtonPressed()==1){
   loopA = 0;
   run++;
   AutonLimelight(0);
 }
   break;
   case 6:
   JoystickDriveOne(1);
 if(Driver.GetAButtonPressed()==1){
   loopA = 0;
    JoystickDriveOne(0);
   run++;
 }
   break;
   case 7:

 JoystickDrive();
 JoystickDriveOne(0);
  AutonLimelight(0);
     nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline",1);

 if(Driver.GetAButtonPressed()==1){
   loopA = 0;
   run++;

 }
 break;
 case 8:

   AutonLimelight(1);
   JoystickDriveOne(0);
 if(targetArea>2.75 or Driver.GetAButtonPressed()==1){
   loopA = 0;
   run++;
   AutonLimelight(0);
 }
   break;
   case 9:
   JoystickDriveOne(1);
 if(Driver.GetAButtonPressed()==1){
   loopA = 0;
    JoystickDriveOne(0);
   run++;
 }
 break;
 case 10:
  JoystickDrive();
 break;
   break;
      }
   }else {
     Robot::StabbyControl();  
  //Robot::JoystickDrive();
  Robot::LiftControl();
  Robot::Followers();
  Robot::SmartDashboard();
  Robot::IntakeControl();
  Robot::RollerClawControl();
 // Robot::Limelight();
    // Default Auto goes here
  }
  
  }

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  loopidee++;
 /* if(loopidee>30){
 StabbyS.Set(true);
    StabbyClaw.Set(true);
  }
  if(loopidee>60){
  loopidee = 0;
  }
  if(loopidee<30){
 StabbyS.Set(false);
    StabbyClaw.Set(false);
  }*/
  if(Driver.GetYButton()==1){
    AutonLimelight(1);
  }else{
    AutonLimelight(0);
    Robot::JoystickDrive();
  }
  frc::SmartDashboard::PutNumber("loopidee",loopidee);
 
   double ypr [3];
    Pigeon.GetYawPitchRoll(ypr);
  frc::SmartDashboard::PutNumber("limelightplace",Limelightplace);
 
  if(Driver.GetXButtonPressed()==1){
     Pigeon.SetYaw(0);
    LeftDrive.SetSelectedSensorPosition(0,0,10);
    RightDrive.SetSelectedSensorPosition(0,0,10);
  }
 // nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline",1);

  
  Robot::StabbyControl();  
  Robot::LiftControl();
  Robot::Followers();
  Robot::SmartDashboard();
  Robot::IntakeControl();
  Robot::RollerClawControl();
  Robot::ClimbControl();

 /* if(Manipulatortwo.GetRawButton(9)==1){
    Climb1.Set(ControlMode::PercentOutput,.5);
  }
  else if(Driver.GetStartButton()==1){
    Climb1.Set(ControlMode::PercentOutput,-.25);
  }else{ 
    Climb1.Set(ControlMode::PercentOutput,.0);}*/

      frc::SmartDashboard::PutNumber("yaw", ypr[0]);

  }
void Robot::ClimbControl(){
if(Manipulatortwo.GetRawButtonPressed(9)==1){
  ClimbOveride = !ClimbOveride;
}
if(ClimbOveride==1 and Manipulatortwo.GetRawButtonPressed(4)==1){
  Climb1.Set(ControlMode::Position,1000);
}
if(ClimbOveride==1 and Manipulatortwo.GetRawButtonPressed(1)==1){
  Climb1.Set(ControlMode::Position,0);
}
if(ClimbOveride==1 and Manipulatortwo.GetRawButton(3)==1){
  ClimbFoot.Set(ControlMode::PercentOutput,.2);
}
if(ClimbOveride==1 and Manipulatortwo.GetRawButtonPressed(2)==1){
 /*SUCTION CODE HERE*/
 
 ClimbBase.Set(frc::DoubleSolenoid::kForward);
 Vacuum.Set(ControlMode::PercentOutput,.35);
 
}
}
void Robot::SmartDashboard(){
  frc::SmartDashboard::PutBoolean("item", Item);
  frc::SmartDashboard::PutNumber("run", 	run);
  //frc::SmartDashboard::PutNumber("state", 	_state);
	//frc::SmartDashboard::PutNumber("Buffer", 	LeftDrive.GetMotionProfileTopLevelBufferCount());
  frc::SmartDashboard::PutNumber("Intake Arm",IntakeArm.GetSelectedSensorPosition(0));
  frc::SmartDashboard::PutNumber("Elevator",Lift.GetSelectedSensorPosition(0));
 frc::SmartDashboard::PutNumber("right encoder",RightDrive.GetSelectedSensorPosition(0));
  }
void Robot::ArcadeDrive(bool Go){
  double ypr [3];
    Pigeon.GetYawPitchRoll(ypr);
  double Anglea = ypr[0];
  
  double offseta = .03*(SetAngle-ypr[0]) ;
  if(Driver.GetX(frc::XboxController::kLeftHand)>.1 or Driver.GetX(frc::XboxController::kLeftHand)<-.1){
    XValue = Driver.GetX(frc::XboxController::kLeftHand);
  }else{
    XValue = 0;
  }
  frc::SmartDashboard::PutNumber("X",Driver.GetX(frc::XboxController::kLeftHand));
  if(loopB == 0){
  SetAngle=ypr[0];
  }else if(loopB>0){
  SetAngle = SetAngle + 1*XValue;
  }
  frc::SmartDashboard::PutNumber("angleset1",SetAngle);
   frc::SmartDashboard::PutNumber("loop1",loopA);
    frc::SmartDashboard::PutNumber("offset1",offseta);
  if(Go == 1){

  LeftDrive.Set(ControlMode::PercentOutput,-Driver.GetY(frc::XboxController::kRightHand)-offseta);
  RightDrive.Set(ControlMode::PercentOutput,-Driver.GetY(frc::XboxController::kRightHand)+offseta);


  loopB++;
    }else{
      loopB = 0;
    }
  };
void Robot::Followers(){ 
  Climb2.Follow(Climb1,FollowerType_PercentOutput);
  LeftDrive1.Follow(LeftDrive, FollowerType_PercentOutput);
	LeftDrive2.Follow(LeftDrive, FollowerType_PercentOutput);
	RightDrive1.Follow(RightDrive, FollowerType_PercentOutput);
	RightDrive2.Follow(RightDrive, FollowerType_PercentOutput);
  RightDrive1.SetInverted(RightDrive.GetInverted());
			RightDrive2.SetInverted(RightDrive.GetInverted());
      LeftDrive1.SetInverted(LeftDrive.GetInverted());
			LeftDrive2.SetInverted(LeftDrive.GetInverted());}
void Robot::RunProfile(BufferedTrajectoryPointStream& Buffer){
   // InitBuffer(Buffer,profile,totalCnt,direction);
    LeftDrive.ClearMotionProfileTrajectories();
    LeftDrive.GetSensorCollection().SetQuadraturePosition(0);
            RightDrive.GetSensorCollection().SetQuadraturePosition(0);
            Pigeon.SetYaw(0);
             RightDrive.Follow(LeftDrive, FollowerType_AuxOutput1);
            LeftDrive.StartMotionProfile(Buffer, 12, ControlMode::MotionProfileArc);
 
  }
void Robot::RollerClawControl(){
  if(ClimbOveride == 0){
    if(Manipulatortwo.GetRawButton(5)==1 and Lift.GetSelectedSensorPosition()<6000){
  RollerClaw.Set(ControlMode::PercentOutput,1);
  }else if(Driver.GetBButton()==1){
  RollerClaw.Set(ControlMode::PercentOutput,-.75);
  }else if(Manipulatortwo.GetRawButton(10)==1){RollerClaw.Set(ControlMode::PercentOutput,1);
  }else if(Item==0){
  RollerClaw.Set(ControlMode::PercentOutput,.2);
  }else{
    RollerClaw.Set(ControlMode::PercentOutput,0);
  }
  }else{
    RollerClaw.Set(ControlMode::PercentOutput,0);
  }
  }
  
  

void Robot::StabbyControl(){
   if(ClimbOveride == 0){
  if(Manipulatortwo.GetRawButtonPressed(4)==1){
   stabbyin = !stabbyin;
  }
   if(Manipulatortwo.GetRawButtonPressed(3)==1){
   finger = !finger;
  }
  if(stabbyin ==1){
    StabbyS.Set(true);
   // StabbyClaw.Set(true);
  }else{
    StabbyS.Set(false);
   //  StabbyClaw.Set(false);
  }
   if(finger ==1){
   // StabbyS.Set(true);
    StabbyClaw.Set(true);
  }else{
   // StabbyS.Set(false);
     StabbyClaw.Set(false);
  }}else{
    StabbyClaw.Set(false);
    StabbyS.Set(false);
  }

 /* else if (Driver.GetAButton()==0){
    StabbyS.Set(false);
     StabbyClaw.Set(false);
  }else if (Driver.GetAButton()==1){
    StabbyS.Set(true);
    StabbyClaw.Set(false);
  }else{
    StabbyS.Set(false);
    StabbyClaw.Set(false);
  }*/
  }
void Robot::IntakeControl(){
  
  if( Manipulatortwo.GetRawButton(5)==1){
    IntakeArm.Set(ControlMode::Position,58000);
    Intake.Set(ControlMode::PercentOutput,.5);
  }
   else{Intake.Set(ControlMode::PercentOutput,0);}

  if(Manipulatortwo.GetRawButton(5)==0 and Lift.GetSelectedSensorPosition(0)<3000 and Lift.GetClosedLoopTarget()<3001 ){
   IntakeArm.Set(ControlMode::Position,000);
  }
 
  else if(Manipulatortwo.GetRawButton(5)==0 and Manipulatortwo.GetRawButton(11)==0 and Lift.GetSelectedSensorPosition(0)>9000 and Lift.GetClosedLoopTarget()>9999  )
  {
    IntakeArm.Set(ControlMode::Position,000);
  }else if(Manipulatortwo.GetRawButton(5)==1){
    IntakeArm.Set(ControlMode::Position,58000);
  }else if(Manipulatortwo.GetRawButton(11)==1){
    IntakeArm.Set(ControlMode::Position,58000);
  }
  else{
    IntakeArm.Set(ControlMode::Position,14000);
  };
  frc::SmartDashboard::PutNumber("11",Manipulatortwo.GetRawButton(11));
 
 
  }

void Robot::JoystickDrive() {
 // double LeftP;
 // double RightP;
 
  if(Driver.GetBumperPressed(frc::XboxController::kRightHand)==1){
  Reverse = !Reverse;
  }
  if(Driver.GetBumper(frc::XboxController::kLeftHand)==1){
    ArcadeDrive(1);
  }
  else if(Reverse==1){
   
    ArcadeDrive(0);
    RightDrive.Set(ControlMode::PercentOutput,1*Driver.GetY(frc::XboxController::kRightHand));
    LeftDrive.Set(ControlMode::PercentOutput,1*Driver.GetY(frc::XboxController::kLeftHand));
    
  }
  else if(Reverse==0){
        ArcadeDrive(0);
   RightDrive.Set(ControlMode::PercentOutput,-1*Driver.GetY(frc::XboxController::kLeftHand));
    LeftDrive.Set(ControlMode::PercentOutput,-1*Driver.GetY(frc::XboxController::kRightHand));
  }

  }
  
void Robot::LiftControl(){
  if(ClimbOveride == 0){
  //0 = cargo 1 = hatch
  Item = Manipulatortwo.GetRawButton(12);
 //hatch low
  if(Item == 1 and Manipulatortwo.GetRawButtonPressed(8)==1){
   ElevatorHeight = 000;
   //Robot::ElevatorSet(2000);
  }

  //hatch medium
   if(Item == 1 and Manipulatortwo.GetRawButtonPressed(7)==1){
   ElevatorHeight = 24000;
   //Robot::ElevatorSet(24000);
  }

  //hatch high
   if(Item == 1 and Manipulatortwo.GetRawButtonPressed(6)==1){
   ElevatorHeight = 46000;
   //Robot::ElevatorSet(46000);
  }
  /*if(Item == 1 and Lift.GetClosedLoopTarget()!=46000 and Lift.GetClosedLoopTarget()!=24000 and Lift.GetClosedLoopTarget()!=2000 and Lift.GetClosedLoopTarget()!=ElevatorHeight){
  
  ElevatorHeight = 000;
  //Robot::ElevatorSet(0000);
  //Lift.Set(ControlMode::PercentOutput,0);
  }*/

  //cargo intake
  if(Item == 0 and Manipulatortwo.GetRawButton(5)==1){
    ElevatorHeight = 5000;
    //Robot::ElevatorSet(5000);
  }else if(Item == 0 and Lift.GetClosedLoopTarget()<5001){
    ElevatorHeight = 10000;
  }

  //cargo low
  if(Item == 0 and Manipulatortwo.GetRawButtonPressed(8)==1){
  ElevatorHeight = 16000;
  //Robot::ElevatorSet(16000);
  }

  //cargo medium
  if(Item == 0 and Manipulatortwo.GetRawButtonPressed(7)==1){
  ElevatorHeight = 38000;
  //Robot::ElevatorSet(38000);
  }

  //cargo high
  if(Item == 0 and Manipulatortwo.GetRawButtonPressed(6)==1){
  ElevatorHeight = 60000;
  //Robot::ElevatorSet(60000);
  }
  
   if(Item == 0 and Manipulatortwo.GetRawButton(10)==1){
  ElevatorHeight = 27000;
  //Robot::ElevatorSet(30000);
  }
  /*if(Item == 0 and Manipulatortwo.GetRawButton(5)==0 and Lift.GetClosedLoopTarget()!=16000 and Lift.GetClosedLoopTarget()!=38000 and Lift.GetClosedLoopTarget()!=60000 and Lift.GetClosedLoopTarget()!=30000 and Lift.GetClosedLoopTarget()!=ElevatorHeight){
 ElevatorHeight = 10000;
 // Robot::ElevatorSet(10000);
  //Lift.Set(ControlMode::PercentOutput,0);
  }*/
  if(Manipulatortwo.GetRawButtonPressed(2)==1){
    ElevatorHeight=ElevatorHeight +2500;
  }
  if(Manipulatortwo.GetRawButtonPressed(1)==1){
    ElevatorHeight=ElevatorHeight-2500;
  }
  if(ElevatorHeight<0){
    ElevatorHeight=0;
  }
  if(IntakeArm.GetSelectedSensorPosition(0)<5500 and Lift.GetClosedLoopTarget()>2501 and Lift.GetSelectedSensorPosition(0)<1000){
    Lift.ConfigPeakOutputForward(0,10);
  }else{
    Lift.ConfigPeakOutputForward(1,10);
  }
  Robot::ElevatorSet(ElevatorHeight);}else{
    Robot::ElevatorSet(ElevatorHeight);
    ElevatorHeight = 0;
  }
  frc::SmartDashboard::PutNumber("elevatorHeight",ElevatorHeight);




  }
void Robot::JoystickDriveOne(bool Go){
   double ypr [3];
    Pigeon.GetYawPitchRoll(ypr);
  double Angle = ypr[0];
  
  double offset = .03*(SetAngle-ypr[0]);
  if(loop == 0){
  SetAngle=ypr[0];
  }else if(loop>0){
  SetAngle = SetAngle;
  }
  frc::SmartDashboard::PutNumber("angleset",SetAngle);
   frc::SmartDashboard::PutNumber("loop",loop);
    frc::SmartDashboard::PutNumber("offset",offset);
  if(Go == 1){
  LeftDrive.Set(ControlMode::PercentOutput,Driver.GetY(frc::XboxController::kRightHand)+offset);
  RightDrive.Set(ControlMode::PercentOutput,Driver.GetY(frc::XboxController::kRightHand)-offset);


  loop++;
    }else{
      loop = 0;
    }
  };
void Robot::Limelight(){
	   std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
      double targetArea = table->GetNumber("ta",0.0); 
   double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
   double DriveSpeed = .025 * targetArea;
   double TurnSpeed = .05 * targetOffsetAngle_Horizontal;
   double InnerTurnSpeed = .07 * targetOffsetAngle_Horizontal;
   double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
 
   double targetSkew = table->GetNumber("ts",0.0);
   double target = table->GetNumber("tv",0.0);
   double RawX0 = table ->GetNumber("tx0",0.0);
   double RawY0 = table ->GetNumber("ty0",0.0);
   double Area0 = table ->GetNumber("ta0",0.0);
   double Skew0 = table ->GetNumber("ts0",0.0);
   double RawX1 = table ->GetNumber("tx1",0.0);
   double RawY1 = table ->GetNumber("ty1",0.0);
   double Area1 = table ->GetNumber("ta1",0.0);
   double Skew1 = table ->GetNumber("ts1",0.0);
   double RawX2 = table ->GetNumber("tx2",0.0);
   double RawY2 = table ->GetNumber("ty2",0.0);
   double Area2 = table ->GetNumber("ta2",0.0);
   double Skew2 = table ->GetNumber("ts2",0.0);

  frc::SmartDashboard::PutNumber("RawX0",RawX0);
   frc::SmartDashboard::PutNumber("RawY0",RawY0);
   frc::SmartDashboard::PutNumber("RawX1",RawX1);
   frc::SmartDashboard::PutNumber("RawY1",RawY1);
   frc::SmartDashboard::PutNumber("Area0",Area0);
   frc::SmartDashboard::PutNumber("Area1",Area1);
   HDistance = (tan((offset*(180/PI))))*(HHieght);
   BDistance = (tan((offset*(180/PI)))*BHieght);
   double eDistance; 
   if (Item == 1){
   eDistance = HDistance*4096/(6*PI);}
   if (Item==0){
    eDistance = BDistance*4096/(6*PI);
   }
   bool LimelightMode;
   frc::SmartDashboard::PutNumber("Horizontal",targetOffsetAngle_Horizontal);
   frc::SmartDashboard::PutNumber("Vertical",targetOffsetAngle_Vertical);
   frc::SmartDashboard::PutNumber("HDistance",HDistance);
   frc::SmartDashboard::PutNumber("TurnSpeed",TurnSpeed);
   //if (Driver.GetBumperPressed(frc::XboxController::kRightHand)==1){
   // RightDrive.Set(ControlMode::Position,eDistance);
    //LeftDrive.Set(ControlMode::Position,eDistance);
   // }
   if (Manipulatortwo.GetRawButton(11)==1){
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode",3);
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode",0);
  
    if(targetArea<5 and targetOffsetAngle_Horizontal>=1 and Manipulatortwo.GetRawButton(11)==1){
       LeftDrive.Set(ControlMode::PercentOutput,TurnSpeed);//Driver.GetY(frc::XboxController::kRightHand)-TurnSpeed);
       RightDrive.Set(ControlMode::PercentOutput,-TurnSpeed);//Driver.GetY(frc::XboxController::kRightHand)+TurnSpeed);
       Robot::RollerClawControl();
       Robot::StabbyControl();  
       Robot::LiftControl();
       Robot::Followers();
       Robot::SmartDashboard();
       Robot::IntakeControl();
    }else if(targetArea<5 and targetOffsetAngle_Horizontal<=-1 and Manipulatortwo.GetRawButton(11)==1){
      LeftDrive.Set(ControlMode::PercentOutput, TurnSpeed);//Driver.GetY(frc::XboxController::kRightHand)-TurnSpeed);
      RightDrive.Set(ControlMode::PercentOutput,-TurnSpeed);//Driver.GetY(frc::XboxController::kRightHand)+TurnSpeed);
      Robot::RollerClawControl();
      Robot::StabbyControl();  
      Robot::LiftControl();
      Robot::Followers();
      Robot::SmartDashboard();
      Robot::IntakeControl();
    }else if(targetArea<5 and targetOffsetAngle_Horizontal<1.5 and targetOffsetAngle_Horizontal>-1.5 and Manipulatortwo.GetRawButton(11)==1){
      LeftDrive.Set(ControlMode::PercentOutput,(Driver.GetY(frc::XboxController::kRightHand)*-.5)+InnerTurnSpeed);
      RightDrive.Set(ControlMode::PercentOutput,(Driver.GetY(frc::XboxController::kRightHand)*-.5)-InnerTurnSpeed);
      Robot::RollerClawControl();
      Robot::StabbyControl();  
      Robot::LiftControl();
      Robot::Followers();
      Robot::SmartDashboard();
      Robot::IntakeControl();
    }
   /* else{
      Robot::RollerClawControl();
      Robot::StabbyControl();  
      Robot::JoystickDrive();
      Robot::LiftControl();
      Robot::Followers();
      Robot::SmartDashboard();
      Robot::IntakeControl();
    }*/
   
    }else{
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode",1);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode",1);
    
      Robot::StabbyControl();  

      Robot::JoystickDrive();
      Robot::LiftControl();
      Robot::Followers();
      Robot::SmartDashboard();
      Robot::IntakeControl();
      Robot::RollerClawControl();
    }

    /*if (targetOffsetAngle_Horizontal<=-2){
    RightDrive.Set(ControlMode::PercentOutput,.25);
    LeftDrive.Set(ControlMode::PercentOutput,-.25);
    }
    if (targetOffsetAngle_Horizontal>= 2){
    RightDrive.Set(ControlMode::PercentOutput,-.25);
    LeftDrive.Set(ControlMode::PercentOutput,.25);}
    else {
     } LeftDrive.Set(ControlMode::PercentOutput,Driver.GetY(frc::XboxController::kLeftHand));
    RightDrive.Set(ControlMode::PercentOutput,Driver.GetY(frc::XboxController::kRightHand));
    if (Manipulatortwo.GetRawButton(1)==!LimelightMode){
   //table
    
    }
   }*/
  }
void Robot::AutonLimelight(bool Go){
	   std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
      double targetArea = table->GetNumber("ta",0.0); 
   double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
   
  
   //double InnerTurnSpeed = .04 * targetOffsetAngle_Horizontal;
   double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
 
   double targetSkew = table->GetNumber("ts",0.0);
   double target = table->GetNumber("tv",0.0);
   double RawX0 = table ->GetNumber("tx0",0.0);
   double RawY0 = table ->GetNumber("ty0",0.0);
   double Area0 = table ->GetNumber("ta0",0.0);
   double Skew0 = table ->GetNumber("ts0",0.0);
   double RawX1 = table ->GetNumber("tx1",0.0);
   double RawY1 = table ->GetNumber("ty1",0.0);
   double Area1 = table ->GetNumber("ta1",0.0);
   double Skew1 = table ->GetNumber("ts1",0.0);
   double RawX2 = table ->GetNumber("tx2",0.0);
   double RawY2 = table ->GetNumber("ty2",0.0);
   double Area2 = table ->GetNumber("ta2",0.0);
   double Skew2 = table ->GetNumber("ts2",0.0);

  frc::SmartDashboard::PutNumber("RawX0",RawX0);
   frc::SmartDashboard::PutNumber("RawY0",RawY0);
   frc::SmartDashboard::PutNumber("RawX1",RawX1);
   frc::SmartDashboard::PutNumber("RawY1",RawY1);
   frc::SmartDashboard::PutNumber("Area0",Area0);
   frc::SmartDashboard::PutNumber("Area1",Area1);
   HDistance = (tan((offset*(180/PI))))*(HHieght);
   BDistance = (tan((offset*(180/PI)))*BHieght);
   double eDistance; 
   if (Item == 1){
   eDistance = HDistance*4096/(6*PI);}
   if (Item==0){
    eDistance = BDistance*4096/(6*PI);
   }
    double TurnSpeed = .04 * targetOffsetAngle_Horizontal;
   double InnerTurnSpeed = .06 * targetOffsetAngle_Horizontal;
   double distanceL = 3.2;
   double DriveSpeed = (.095 / targetArea)/1;//.095
   double speedL = DriveSpeed;
   bool LimelightMode;
   double comparison = abs(Area1 - Area0);
   frc::SmartDashboard::PutNumber("Horizontal",targetOffsetAngle_Horizontal);
   frc::SmartDashboard::PutNumber("Vertical",targetOffsetAngle_Vertical);
   frc::SmartDashboard::PutNumber("HDistance",HDistance);
   frc::SmartDashboard::PutNumber("TurnSpeed",TurnSpeed);
   //if (Driver.GetBumperPressed(frc::XboxController::kRightHand)==1){
   // RightDrive.Set(ControlMode::Position,eDistance);
    //LeftDrive.Set(ControlMode::Position,eDistance);
   // }
   if (Go==1){
   nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode",3);
   nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode",0);
  if(target==1){
    if(targetArea<distanceL and targetOffsetAngle_Horizontal>=2.5 ){
      Limelightplace = 0;
       LeftDrive.Set(ControlMode::PercentOutput,-speedL-TurnSpeed);//Driver.GetY(frc::XboxController::kRightHand)-TurnSpeed);
       RightDrive.Set(ControlMode::PercentOutput,-speedL+TurnSpeed);//Driver.GetY(frc::XboxController::kRightHand)+TurnSpeed);
       Robot::RollerClawControl();
       Robot::StabbyControl();  
       Robot::LiftControl();
       Robot::Followers();
       Robot::SmartDashboard();
       Robot::IntakeControl();
    }else if(targetArea<distanceL and targetOffsetAngle_Horizontal<=-2.5 ){
      Limelightplace = 1;
      LeftDrive.Set(ControlMode::PercentOutput,-speedL -TurnSpeed);//Driver.GetY(frc::XboxController::kRightHand)-TurnSpeed);
      RightDrive.Set(ControlMode::PercentOutput,-speedL+TurnSpeed);//Driver.GetY(frc::XboxController::kRightHand)+TurnSpeed);
      Robot::RollerClawControl();
      Robot::StabbyControl();  
      Robot::LiftControl();
      Robot::Followers();
      Robot::SmartDashboard();
      Robot::IntakeControl();
    }else if(targetArea>3.4){
 LeftDrive.Set(ControlMode::Velocity,0);//Driver.GetY(frc::XboxController::kRightHand)-TurnSpeed);
       RightDrive.Set(ControlMode::Velocity,0);
       Limelightplace=6;
    }else if(targetArea<distanceL and targetOffsetAngle_Horizontal<2.5 and targetOffsetAngle_Horizontal>-2.5 ){
      Limelightplace = 2;
      LeftDrive.Set(ControlMode::PercentOutput,-speedL-TurnSpeed);
      RightDrive.Set(ControlMode::PercentOutput,-speedL+TurnSpeed);
      Robot::RollerClawControl();
      Robot::StabbyControl();  
      Robot::LiftControl();
      Robot::Followers();
      Robot::SmartDashboard();
      Robot::IntakeControl();
    }else if (comparison>=700 and targetOffsetAngle_Horizontal<2.5 and targetOffsetAngle_Horizontal>-2.5){
      Limelightplace = 3;
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode",3);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode",0);
    
      Robot::StabbyControl();  

      Robot::JoystickDrive();
      Robot::LiftControl();
      Robot::Followers();
      Robot::SmartDashboard();
      Robot::IntakeControl();
      Robot::RollerClawControl(); 
    }}else if(targetArea>distanceL+1 and (targetOffsetAngle_Horizontal>.5 or targetOffsetAngle_Horizontal<-.5)){
      Limelightplace = 4;
      LeftDrive.Set(ControlMode::PercentOutput,-1*InnerTurnSpeed - 0.1);
      RightDrive.Set(ControlMode::PercentOutput,1*InnerTurnSpeed + .1);

    }else{
      Limelightplace = 5;
       LeftDrive.Set(ControlMode::Velocity,0);//Driver.GetY(frc::XboxController::kRightHand)-TurnSpeed);
       RightDrive.Set(ControlMode::Velocity,0);
    }
    }else{
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode",1);
   nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode",1);
      // LeftDrive.Set(ControlMode::Velocity,0);//Driver.GetY(frc::XboxController::kRightHand)-TurnSpeed);
      // RightDrive.Set(ControlMode::Velocity,0);//Driver.Get
    }
  };
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
void Robot::ElevatorSet(int Position){
  Lift.Set(ControlMode::Position,Position);
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
void Robot::TestPeriodic() {

  }

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
