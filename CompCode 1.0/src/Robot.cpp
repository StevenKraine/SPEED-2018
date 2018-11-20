/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <fstream>
#include <iostream>
#include <string>
#include <WPILib.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <math.h>
#include <MotionProfile.h>
#include <MotionProfileController.h>
#include <MotionProfileControllerR.h>
#include "MotionProfile1R.h"
#include "MotionProfileController1.h"
//#include <MotionProfileBackR.h>
//#include <MotionProfileBack.h>

class Robot : public frc::IterativeRobot {
	DigitalInput Banner;
	XboxController Driver;
	XboxController Manipulator;
	WPI_TalonSRX LeftDrive;
	WPI_TalonSRX RightDrive;

	WPI_VictorSPX LeftDrive2;
	WPI_VictorSPX RightDrive1;
	WPI_VictorSPX RightDrive2;
	WPI_VictorSPX LeftDrive1;
	WPI_TalonSRX Lift;

	WPI_VictorSPX Lift1;

	WPI_VictorSPX IntakeR;
	WPI_VictorSPX IntakeL;


	PowerDistributionPanel PDP;
	Solenoid Intake1;
	DoubleSolenoid SpeedB;
	Compressor presor;



	ADXRS450_Gyro Gyro;

	//DifferentialDrive Drive;

public:
	// std::ofstream RecordL;
	//std::ofstream RecordR;
				 MotionProfileController1 ProfileL;

				 		MotionProfileController1R ProfileR;


	bool Foot = 0;
	bool DriveDirection = 0;
	double LoopCount = 0;
	bool TurnDriveDirectionOn=0;
	double OneDirection = -1;
	bool RightTurn = 0;
	bool Enabled = 1;
	bool Mototron = 0;
	int Status=1;
	bool IntakeUp = 0;
	double IntakeSpeed = 0;
	int AutonStatus = 0;
	double LeftVel = 0;
	double RightVel = 0;
	double LeftRot = 0;
	double RightRot = 0;
	bool Low = 0;
	bool High = 0;
	bool UltraHigh = 0;
	 bool banner = Banner.Get();
	 int PositionL = 0;
	int PositionR =0;





	Robot(void):

	Banner (0),
	Driver (0),
	Manipulator (1),
	LeftDrive(3),
	RightDrive (2),
	RightDrive1(6),
	RightDrive2(7),
	LeftDrive1(9),
	LeftDrive2(8),
	Lift(1),
	Lift1(11),
//	IntakeWrist(8),
	IntakeR(4),
	IntakeL(5),
	//LiftRelease(11),
	PDP(39),

	Intake1 (0,0),
	SpeedB (0,1,2),
	presor (0),
	Gyro (SPI::kOnboardCS0),
	ProfileL (LeftDrive),
		ProfileR (RightDrive)




{}
	void RobotInit() {



	//	m_chooser.AddDefault(kLeft, kLeft);
	//	m_chooser.AddObject(kRight, kRight);
	//	m_chooser.AddObject(kCenter, kCenter);
	//	m_chooser.AddObject(kStraight, kStraight);
	//	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
		//Left Drive
		LeftDrive.Config_kF(0,.5,10);
		LeftDrive.EnableCurrentLimit(true);
		LeftDrive.ConfigOpenloopRamp(.125,10);
		LeftDrive.ConfigPeakCurrentLimit(30,10);
		LeftDrive.ConfigPeakCurrentDuration(2000,10);
		LeftDrive.Config_kP(0,.080,10);
		LeftDrive.Config_kI(0,.01,10);
		LeftDrive.Config_kD(0,15,10);

		LeftDrive.Config_IntegralZone(0,100,10);
		LeftDrive.ConfigSelectedFeedbackSensor(QuadEncoder,0,10);
		LeftDrive.SetSensorPhase(true);
		LeftDrive.SetNeutralMode(NeutralMode::Coast);
		LeftDrive.SetInverted(false);

		LeftDrive1.SetInverted(LeftDrive.GetInverted());
		LeftDrive2.SetInverted(LeftDrive.GetInverted());



		LeftDrive.ConfigPeakOutputForward(1,10);
		LeftDrive.ConfigPeakOutputReverse(-1,10);
		//RightDrive
		RightDrive.SetInverted(true);
		RightDrive.ConfigPeakOutputForward(1,10);
		RightDrive.ConfigPeakOutputReverse(-1,10);
		RightDrive1.SetInverted(RightDrive.GetInverted());
		RightDrive2.SetInverted(RightDrive.GetInverted());
		RightDrive.EnableCurrentLimit(true);
		RightDrive.ConfigOpenloopRamp(.125,10);
		RightDrive.ConfigPeakCurrentLimit(30,10);
		RightDrive.ConfigPeakCurrentDuration(2000,10);
		RightDrive.Config_kP(0,.08,10);
		RightDrive.Config_kI(0,.01,10);
		RightDrive.Config_kD(0,15,10);

		RightDrive.Config_IntegralZone(0,100,10);
		RightDrive.ConfigSelectedFeedbackSensor(QuadEncoder,0,10);
		RightDrive.SetSensorPhase(true);
		RightDrive.SetNeutralMode(NeutralMode::Coast);

		RightDrive.ConfigMotionProfileTrajectoryPeriod(10,10);
	//	RightDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic,
		//10, Constants::kTimeoutMs);

		Lift1.SetInverted(true);

		//Lift
		Lift.EnableCurrentLimit(true);
		Lift.ConfigPeakCurrentLimit(2,10);
		Lift.ConfigPeakCurrentDuration(10000,10);
		Lift.Config_kP(0,1.3,10);
		Lift.Config_kI(0,.00125,10);
		Lift.Config_kD(0,220,10);
		Lift.ConfigSelectedFeedbackSensor(QuadEncoder,0,10);
		Lift.ConfigOpenloopRamp(1,10);
		Lift.ConfigClosedloopRamp(.25,10);
		Lift.SetNeutralMode(NeutralMode::Coast);
		Lift1.SetNeutralMode(NeutralMode::Coast);
		Lift.SetSensorPhase(true);
		//IntakeWrist

		//EverythingElse
		LeftDrive1.SetNeutralMode(NeutralMode::Coast);
		LeftDrive2.SetNeutralMode(NeutralMode::Coast);
		RightDrive1.SetNeutralMode(NeutralMode::Coast);
		RightDrive2.SetNeutralMode(NeutralMode::Coast);

		IntakeR.ConfigOpenloopRamp(.06,10);
		IntakeL.ConfigOpenloopRamp(.06,10);
		IntakeR.SetInverted(true);
		IntakeL.SetInverted(!IntakeR.GetInverted());







	}

	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */
	void AutonomousInit() override {
		std::string gameData;
				gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		m_autoSelected = m_chooser.GetSelected();
		 m_autoSelected = SmartDashboard::GetString(
		 		"Auto Selector", kAutoNameDefault);
	std::cout << "Auto selected: " << m_autoSelected << std::endl;
		LeftDrive.SetSelectedSensorPosition(0,0,10);
		RightDrive.SetSelectedSensorPosition(0,0,10);
do{if (m_autoSelected == kLeft) {
	if (gameData[0]=='L' and gameData[1]=='L'){
				AutonStatus = 1;
			}else if(gameData[0]=='L' and gameData[1]=='R'){
				AutonStatus = 20;
			}else if(gameData[0]=='R' and gameData[1]=='R'){
				AutonStatus = 30;
			}else if(gameData[0]=='R' and gameData[1]=='L'){
				AutonStatus = 40;
			}

} else if(m_autoSelected == kCenter){
	if (gameData[0]=='L' and gameData[1]=='L'){
			AutonStatus = 1;
		}else if(gameData[0]=='L' and gameData[1]=='R'){
			AutonStatus = 20;
		}else if(gameData[0]=='R' and gameData[1]=='R'){
			AutonStatus = 30;
		}else if(gameData[0]=='R' and gameData[1]=='L'){
			AutonStatus = 40;
		}
}else if(m_autoSelected == kRight){
	if (gameData[0]=='L' and gameData[1]=='L'){
				AutonStatus = 1;
			}else if(gameData[0]=='L' and gameData[1]=='R'){
				AutonStatus = 20;
			}else if(gameData[0]=='R' and gameData[1]=='R'){
				AutonStatus = 30;
			}else if(gameData[0]=='R' and gameData[1]=='L'){
				AutonStatus = 40;
			}
}else if (m_autoSelected == kStraight){
	if (gameData[0]=='L' and gameData[1]=='L'){
				AutonStatus = 1;
			}else if(gameData[0]=='L' and gameData[1]=='R'){
				AutonStatus = 20;
			}else if(gameData[0]=='R' and gameData[1]=='R'){
				AutonStatus = 30;
			}else if(gameData[0]=='R' and gameData[1]=='L'){
				AutonStatus = 40;
			}
}}while(AutonStatus == 0);

	}

	void AutonomousPeriodic() {
		SmartDashboard::PutNumber("Left Drive Encoder",LeftDrive.GetSelectedSensorPosition(0));
				SmartDashboard::PutNumber("Right Drive Encoder",RightDrive.GetSelectedSensorPosition(0));
 SmartDashboard::PutNumber("AutonStatus",AutonStatus);
		LeftDrive1.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
				LeftDrive2.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
				RightDrive1.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());
				RightDrive2.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());

				IntakeL.Set(ControlMode::PercentOutput,IntakeSpeed);
				IntakeR.Set(ControlMode::PercentOutput,IntakeSpeed);

				RightDrive.Set(ControlMode::Position,PositionR);
		if (m_autoSelected == kLeft) {
			switch(AutonStatus){
			case(1):
			LeftDrive.Set(ControlMode::Position,40000);

			PositionR =40000;
			SpeedB.Set(DoubleSolenoid::kReverse);
			IntakeSpeed = .1;
			if(PositionL >= 40000 and PositionR >= 40000){

			}
			break;
			case (2):
			break;
			}
		} else if(m_autoSelected == kRight){

		} else if(m_autoSelected == kCenter){

		}else if(m_autoSelected == kStraight){
					}
	}

	void TeleopInit() {


		 LeftDrive.SetSelectedSensorPosition(0,0,10);
		 RightDrive.SetSelectedSensorPosition(0,0,10);

		Gyro.Reset();
		 LoopCount = 0;
			//	 RightDrive.SetInverted(!RightDrive.GetInverted());


	}
 void LiftControl(){

	 if(Manipulator.GetAButtonPressed() == 1){
		Low = !Low;
		High = 0;
		UltraHigh = 0;
	}
	if(Manipulator.GetXButtonPressed() == 1){
		Low = 0;
		High = !High;
		UltraHigh = 0;
	}
	if(Manipulator.GetYButtonPressed() == 1){
			Low = 0;
			High = 0;
			UltraHigh = !UltraHigh;
		}
	if(SpeedB.Get()== 2 and Low == 1){
		Lift.Set(ControlMode::Position,12000);
	}
	if (SpeedB.Get()== 2 and High == 1){
			Lift.Set(ControlMode::Position,17000);
		}
	if (SpeedB.Get()== 2 and UltraHigh == 1){
			Lift.Set(ControlMode::Position,19000);

	}
	if (UltraHigh != 1 and High !=1 and Low !=1 ){
		Lift.Set(ControlMode::Position,0);
	}


 }
 void IntakeControl(){
		if(Manipulator.GetBumper(XboxController::kLeftHand)==1 and Driver.GetAButton()!=1 and Driver.GetYButton()!=1){
				IntakeSpeed = .75;
			}else if (Driver.GetAButton()==1 and Manipulator.GetBumper(XboxController::kLeftHand)!=1 and Driver.GetYButton()!= 1){
				IntakeSpeed = -1;
			}else if (Driver.GetAButton()!=1 and Manipulator.GetBumper(XboxController::kLeftHand)!=1 and Driver.GetYButton()== 1){
				IntakeSpeed = -.25;
			}else {IntakeSpeed = .1;}
 }
 void TeleopPeriodic() {
		 LiftControl();
		 IntakeControl();
/*		 std::shared_ptr<NetworkTable> table =   NetworkTable::GetTable("limelight");
		 float targetOffsetAngle_Horizontal = table->GetNumber("tx",0);
		 float targetOffsetAngle_Vertical = table->GetNumber("ty",0);
		 float targetArea = table->GetNumber("ta",0);
		 float targetSkew = table->GetNumber("ts",0);

		 table->PutNumber("camMode",1);
		 table->PutNumber("ledMode",1);*/

				LeftVel = LeftDrive.GetSelectedSensorVelocity(0);
		RightVel = RightDrive.GetSelectedSensorVelocity(0);
		LeftRot = LeftDrive.GetSelectedSensorPosition(0);
		RightRot = RightDrive.GetSelectedSensorPosition(0);

		double leftVel = LeftVel/4096 * 600;
		double rightVel = RightVel/4096 *600;
		double leftRot = LeftRot/4096;
		double rightRot = RightRot/4096;

	//	  RecordL.open ("/home/lvuser/Recordings/Recordings.csv", std::ofstream::out | std::ofstream::app);
	//			  RecordR.open ("/home/lvuser/Recordings/RecordingsR.csv", std::ofstream::out | std::ofstream::app);



	//	RecordL << "{"<<leftRot<<","<<leftVel<<",10},"<< "\n";
  	//	RecordR << "{"<<rightRot<<","<<rightVel<<",10},"<< "\n";

//RecordL.close();
//RecordR.close();



		Lift1.Set(ControlMode::PercentOutput,Lift.GetMotorOutputPercent());



		int control = ProfileL.getSetValue();

		double gyro= Gyro.GetAngle();

		presor.SetClosedLoopControl(true);
	/*	LeftDrive1.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
		LeftDrive2.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
		RightDrive1.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());
		RightDrive2.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());
		LeftDrive.Set(ControlMode::PercentOutput,-1* Driver.GetY(XboxController::kLeftHand));
		RightDrive.Set(ControlMode::PercentOutput,-1*Driver.GetY(XboxController::kRightHand));
		IntakeL.Set(ControlMode::PercentOutput,IntakeSpeed);
		IntakeR.Set(ControlMode::PercentOutput,IntakeSpeed);*/

		LoopCount = LoopCount +1;



	




		/*if(Status == 1){
			SetValueMotionProfile setOutput = ProfileL.getSetValue();


		//	LeftDrive.Set(ControlMode::MotionProfile,1);
			LeftDrive.Set(ControlMode::MotionProfile,setOutput);
			ProfileL.start();
		//	RightDrive.Set(ControlMode::MotionProfile,1);
			RightDrive.Set(ControlMode::MotionProfile,setOutput);
			ProfileR.start();*/
		/*LeftDrive.Set(ControlMode::MotionProfile,1);
				ProfileBL.start();
				RightDrive.Set(ControlMode::MotionProfile,1);
				ProfileBR.start();*/

































		SmartDashboard::PutNumber("left output",LeftDrive.GetMotorOutputPercent());
		SmartDashboard::PutNumber("Amps from Lift",Lift.GetOutputCurrent());
		SmartDashboard::PutNumber("Right output",RightDrive.GetMotorOutputPercent());
		SmartDashboard::PutNumber("SetOutput",ProfileL.getSetValue());

		SmartDashboard::PutNumber("Elavator Encoder",Lift.GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("Status",Status);
		SmartDashboard::PutNumber("Banner",Banner.Get());
		SmartDashboard::PutNumber("RightTurn",RightTurn);
		
		SmartDashboard::PutNumber("Gyro",gyro);
		SmartDashboard::PutNumber("RightDrive",RightDrive.GetMotorOutputVoltage());
		SmartDashboard::PutNumber("LeftDrive",LeftDrive.GetMotorOutputVoltage());
		SmartDashboard::PutNumber("RightFollower1",RightDrive1.GetMotorOutputVoltage());
		SmartDashboard::PutNumber("LeftFollower1",LeftDrive1.GetMotorOutputVoltage());
		SmartDashboard::PutNumber("Foot",Foot);
		SmartDashboard::PutNumber("PDP Voltage",PDP.GetVoltage());
		SmartDashboard::PutNumber("PDP Temp",PDP.GetTemperature()*(9/5)+32);
		SmartDashboard::PutNumber("Left Drive Encoder",LeftDrive.GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("Right Drive Encoder",RightDrive.GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("Right Joystick Y",Driver.GetY(XboxController::kRightHand));
		SmartDashboard::PutNumber("Left Joystick Y", Driver.GetY(XboxController::kLeftHand));
		SmartDashboard::PutNumber("Enabled",Enabled);
		SmartDashboard::PutNumber("Velocity",LeftDrive.GetSelectedSensorVelocity(0));
		SmartDashboard::PutNumber("Velocity1",RightDrive.GetSelectedSensorVelocity(0));
		
	}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kLeft = "Left";
	const std::string kRight = "Right";
	const std::string kCenter = "Center";
	const std::string kStraight = "Straight";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
