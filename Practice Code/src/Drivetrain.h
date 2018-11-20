/*
 * Drivetrain.h
 *
 *  Created on: Mar 19, 2018
 *      Author: steven
 */

#ifndef SRC_DRIVETRAIN_H_
#define SRC_DRIVETRAIN_H_
using namespace std;
#include <fstream>
#include <iostream>
#include <string>
#include <WPILib.h>
#include <IterativeRobot.h>
#include <llvm/raw_ostream.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include <networktables/NetworkTable.h>
#include <iomanip>
#include <HAL/HAL.h>
#include <thread>
#include <math.h>
class drivetrain{
private:
	int loopcount1 = 0;
	bool inPos = 0;
	double PositionR = 0;
		double PositionL = 0;
		double VelR = 0;
		double VelL = 0;
		double VelocityCorrection = 0;
		double LeftRotations =LeftDrive.GetSelectedSensorPosition(0);
		double velocityL = 0;
		double velocity = 0;
		double RightRotations = 0;
		double PositionOffset = 0;
		double kP = .1;//.1
		double kI = .5;//.5??
		double kD = 20;//20
		double kF = 0;
	double Target = 0;
	bool Straight = 0;

public:

	WPI_TalonSRX LeftDrive;
	WPI_TalonSRX RightDrive;
	WPI_VictorSPX RightDrive1;
	WPI_VictorSPX RightDrive2;
	WPI_VictorSPX LeftDrive1;
	WPI_VictorSPX LeftDrive2;


		//Const int



	 drivetrain(void):
		 LeftDrive(3),//3
		 RightDrive (2),
		 RightDrive1(6),
		 RightDrive2(7),
		 LeftDrive1(9),//9//
		 LeftDrive2(8)

	 {}
	 void DriveTrainInit(){

			LeftDrive.SetInverted(true);
			LeftDrive1.SetInverted(LeftDrive.GetInverted());
			LeftDrive2.SetInverted(LeftDrive.GetInverted());
			LeftDrive.ConfigSelectedFeedbackSensor(QuadEncoder,0,10);
			LeftDrive.Config_kP(0,kP,10);//.1
			LeftDrive.Config_kI(0,kI,10);//.5
			LeftDrive.Config_kD(0,kD,10);//20
			LeftDrive.Config_kF(0,kF,10);
			LeftDrive.Config_kF(0,0,10);
			LeftDrive.Config_IntegralZone(0,20,10);
			LeftDrive.EnableCurrentLimit(true);
			LeftDrive.ConfigOpenloopRamp(.125,10);
			LeftDrive.ConfigPeakCurrentLimit(30,10);
			LeftDrive.ConfigPeakCurrentDuration(2000,10);
			LeftDrive.SetNeutralMode(NeutralMode::Coast);
			LeftDrive1.SetNeutralMode(NeutralMode::Brake);
			LeftDrive2.SetNeutralMode(NeutralMode::Brake);
			LeftDrive.SetSensorPhase(true);
			LeftDrive.ConfigClosedloopRamp(0.5,10);
			LeftDrive.ConfigMotionProfileTrajectoryPeriod(40,10);

			RightDrive.ConfigMotionProfileTrajectoryPeriod(40,10);
			RightDrive.SetInverted(false);
			RightDrive1.SetInverted(RightDrive.GetInverted());
			RightDrive2.SetInverted(RightDrive.GetInverted());
			RightDrive.ConfigSelectedFeedbackSensor(QuadEncoder,0,10);
			RightDrive.Config_kP(0,kP,10);//.1
			RightDrive.Config_kI(0,kI,10);//.5
			RightDrive.Config_kD(0,kD,10);//20
			RightDrive.Config_kF(0,kF,10);

			RightDrive.Config_IntegralZone(0,20,10);
			RightDrive.EnableCurrentLimit(true);
			RightDrive.ConfigOpenloopRamp(0.125,10);
			RightDrive.ConfigClosedloopRamp(0.125,10);
			RightDrive.ConfigPeakCurrentLimit(30,10);
			RightDrive.ConfigPeakCurrentDuration(2000,10);
			RightDrive.SetNeutralMode(NeutralMode::Coast);
			RightDrive1.SetNeutralMode(NeutralMode::Brake);
			RightDrive2.SetNeutralMode(NeutralMode::Brake);
			RightDrive.SetSensorPhase(true);

	 }
	//distance is in inches
	 void STOP(){

		 LeftDrive.ConfigPeakOutputForward(0,10);

			 				 			 				RightDrive.ConfigPeakOutputForward(0,10);
			 				 			 				LeftDrive.ConfigPeakOutputReverse(0,10);
			 				 			 						RightDrive.ConfigPeakOutputReverse(0,10);
		 LeftDrive.Set(ControlMode::Current,0);
		 RightDrive.Set(ControlMode::Current,0);
		 FollowCommand();
	 }
	 void DriveStraight(double Distance, double MAX){
		  kP = .1;//.1
		 		 kI = .5;//.5??
		 		 kD = 20;
		 LeftDrive.ConfigPeakOutputForward(MAX,10);
		 RightDrive.ConfigPeakOutputForward(MAX+.1,10);
		 LeftDrive.ConfigPeakOutputReverse(-1*MAX,10);
		 RightDrive.ConfigPeakOutputReverse(-1*MAX-.1,10);
		 LeftRotations =LeftDrive.GetSelectedSensorPosition(0);

		 velocityL = LeftDrive.GetSelectedSensorVelocity(0);
		 velocity = RightDrive.GetSelectedSensorVelocity(0);
		 RightRotations = RightDrive.GetSelectedSensorPosition(0);
		 VelL = (velocityL)/4096*600;
		 VelR = (velocity)/4096*600;
		 PositionR = (RightRotations/4096)*1.0;
		 PositionL = (LeftRotations/4096)*1.0;

		 VelocityCorrection = (velocityL-velocity)/5000;
		 PositionOffset = (LeftRotations-RightRotations)/2250;
		// LeftDrive.
		 LeftDrive.Set(ControlMode::Position,Distance*-339.463817756);
		 //double Offset = (RightDrive.GetSelectedSensorPosition(0)-LeftDrive.GetSelectedSensorPosition(0));

		 if(LeftDrive.GetMotorOutputPercent()==0 and LeftDrive.GetSelectedSensorPosition(0)>-1000){
			 RightDrive.Set(ControlMode::PercentOutput,-1);
		 }else{
			 RightDrive.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent() +PositionOffset+VelocityCorrection);
		 }
		 FollowCommand();
		 Target = fabs(Distance);
	 		Straight = 1;
	 			 	};
	 void DriveArc(double StartPercent,double EndPercent,double ArcLength,double Heading,double loopcount){

		 double loopcount1 ;
		 double LeftPosInit;
		 double LeftPos;
		 double RightPosInit;
		 double RightPos;
		 double NativeUnitsArcLength;
		double EncoderHeadingDifference;
		double PercentOfTurn;

		double Modifier;
		NativeUnitsArcLength = (ArcLength *-339.463817756);
		LeftPos = fabs(abs(LeftDrive.GetSelectedSensorPosition(0))-fabs(LeftPosInit));
		RightPos = fabs(abs(RightDrive.GetSelectedSensorPosition(0))-fabs(RightPosInit));

				EncoderHeadingDifference= ((fabs(Heading))*153.6)*PercentOfTurn;
				Modifier = EncoderHeadingDifference*10;
		if(loopcount1 == 0){
			LeftPosInit = LeftDrive.GetSelectedSensorPosition(0);
			RightPosInit = RightDrive.GetSelectedSensorPosition(0);
		}


		 if(Heading <0){


			 PercentOfTurn = ((fabs(NativeUnitsArcLength)) - LeftPos)/fabs(NativeUnitsArcLength);
			 LeftDrive.Set(ControlMode::PercentOutput,StartPercent);
			 RightDrive.Set(ControlMode::PercentOutput,StartPercent-(Modifier));
		 }else if(Heading>0){


			 PercentOfTurn = ((fabs(NativeUnitsArcLength)) - RightPos)/fabs(NativeUnitsArcLength);
			 RightDrive.Set(ControlMode::PercentOutput,StartPercent);
			 LeftDrive.Set(ControlMode::PercentOutput,StartPercent-(Modifier));
		 }
		if(loopcount==0){
					loopcount1 = loopcount1 +1;
				}
		FollowCommand();
		Target = fabs(ArcLength);
		Straight = 1;
	 }
	 void DriveStaightWimpy(double Percent,double MAX2,double Distance1){
		 LeftDrive.ConfigPeakOutputForward(MAX2,10);
		 RightDrive.ConfigPeakOutputForward(MAX2+.1,10);
		 LeftDrive.ConfigPeakOutputReverse(-1*MAX2,10);
		 RightDrive.ConfigPeakOutputReverse(-1*MAX2-.1,10);
		 LeftDrive.Set(ControlMode::PercentOutput,Percent);
		 LeftRotations =LeftDrive.GetSelectedSensorPosition(0);

		 velocityL = LeftDrive.GetSelectedSensorVelocity(0);
		 velocity = RightDrive.GetSelectedSensorVelocity(0);
		 RightRotations = RightDrive.GetSelectedSensorPosition(0);
		 VelL = (velocityL)/4096*600;
		 VelR = (velocity)/4096*600;
		 PositionR = (RightRotations/4096)*1.0;
		 PositionL = (LeftRotations/4096)*1.0;

		 VelocityCorrection = (velocityL-velocity)/5000;
		 PositionOffset = (LeftRotations-RightRotations)/2250;

		Straight = 1;
		RightDrive.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent() +PositionOffset+VelocityCorrection);
		Target = fabs(Distance1);
	 }
	 bool InPosition(double Tolerance){
		 double Error;
		 if(Straight == 1){
			Error  = (fabs(((LeftDrive.GetSelectedSensorPosition(0)+RightDrive.GetSelectedSensorPosition(0))/2)/339.463817756))-Target;
		 }else{
			 Error = ((abs(LeftDrive.GetSelectedSensorPosition(0))+(abs(RightDrive.GetSelectedSensorPosition(0))))/2/76.8)-Target;
		 }

		if(Error<0){
			Error = Error * -1;
		}
		 if(Error <= Tolerance){
			 inPos = 1;
			 return 1;

		 }else{
			 inPos = 0;
			 return 0;
		 }


	 }
	 void HOLD(){
		LeftDrive.Set(ControlMode::MotionProfile,2);
		RightDrive.Set(ControlMode::MotionProfile,2);
	 }
	 void reset(){
		 LeftDrive.SetSelectedSensorPosition(0,0,10);
		 RightDrive.SetSelectedSensorPosition(0,0,10);
		 inPos = 0;
	 }
	 	// Positive is a right Turn in degrees
	 	void DriveTurn(double Degrees, double MAX1){
	 		 LeftDrive.ConfigPeakOutputForward(MAX1,10);
	 				 			 				RightDrive.ConfigPeakOutputForward(MAX1+.1,10);
	 				 			 				LeftDrive.ConfigPeakOutputReverse(-1*MAX1,10);
	 				 			 						RightDrive.ConfigPeakOutputReverse(-1*MAX1-.1,10);
	 		 LeftRotations =LeftDrive.GetSelectedSensorPosition(0);

	 									 		 velocityL = LeftDrive.GetSelectedSensorVelocity(0);
	 									 		 velocity = RightDrive.GetSelectedSensorVelocity(0);
	 									 		 RightRotations = RightDrive.GetSelectedSensorPosition(0);
	 									 		VelL = (velocityL)/4096*600;
	 									 		VelR = (velocity)/4096*600;
	 									 		PositionR = (RightRotations/4096)*1.0;
	 									 		PositionL = (LeftRotations/4096)*1.0;

	 									 		VelocityCorrection = (velocityL+velocity)/5000;
	 									 		 PositionOffset = (LeftRotations+RightRotations)/2250;
	 		double NativeUnits = (Degrees * 76.8 * 1.5);
	 		LeftDrive.Set(ControlMode::Position,-1*NativeUnits);
	 		//RightDrive.Set(ControlMode::Position,NativeUnits);
	 		 //if(LeftDrive.GetMotorOutputPercent()==0 and LeftDrive.GetSelectedSensorPosition(0)>-1000){
	 			 				 		// 				RightDrive.Set(ControlMode::PercentOutput,-1);
	 			 				 		 		//	}else{
	 			 				 		 				RightDrive.Set(ControlMode::PercentOutput,-1*(LeftDrive.GetMotorOutputPercent() +PositionOffset+VelocityCorrection));
	 			 				 		 		//	}
	 			 				 		 			if(inPos == 1){
	 			 				 		 			LeftDrive.Set(ControlMode::PercentOutput,0);
													RightDrive.Set(ControlMode::PercentOutput,0);

	 			 				 		 				 		}
	 			 	if(fabs(Degrees)<45)
	 			 	{
	 			 		 kP = .1;//.1
	 			 				 kI = .5;//.5??
	 			 				 kD = 20;
	 			 	}else if(fabs(Degrees)<90 and fabs(Degrees)>45)
	 			 	{
	 			 		 kP = .1;//.1
	 			 				 kI = .5;//.5??
	 			 				kD = 20;
	 			 	}else if(fabs(Degrees)<135 and fabs(Degrees)>90)
	 			 	{
	 			 		 kP = .1;//.1
	 			 			 kI = .5;//.5??
	 			 				 kD = 20;
	 			 	}else if(fabs(Degrees)<180 and fabs(Degrees)>135)
	 			 	{
	 			 		 kP = .1;//.1
	 			 				 kI = .5;//.5??
	 			 				 kD = 20;
	 			 	}
	 		FollowCommand();
	 		Target = fabs(Degrees);
	 		Straight = 0;

	 	};

	 	void FollowCommand(){
	 		LeftDrive1.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
	 		LeftDrive2.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
	 		RightDrive1.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());
	 		RightDrive2.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());
	 	};
	 	void TeleopDrive (double ThrottleMax,bool Inverted, double LeftJoy, double RightJoy ){
	 		DriveTrain_Tele_Limits();
	 		if(Inverted == 0 ){
	 			LeftDrive.Set(ControlMode::PercentOutput,LeftJoy * ThrottleMax);
	 			RightDrive.Set(ControlMode::PercentOutput,RightJoy * ThrottleMax);
	 		}else if (Inverted == 1){
	 			LeftDrive.Set(ControlMode::PercentOutput,-1*RightJoy * ThrottleMax);
	 			RightDrive.Set(ControlMode::PercentOutput,-1*LeftJoy * ThrottleMax);
	 		}

	 		FollowCommand();
	 	}

	 	void DriveTrain_Tele_Limits(){
	 				LeftDrive.ConfigPeakOutputForward(1,10);
	 				RightDrive.ConfigPeakOutputForward(1,10);
	 				LeftDrive.ConfigPeakOutputReverse(-1,10);
	 				RightDrive.ConfigPeakOutputReverse(-1,10);
	 	}
};







#endif /* SRC_DRIVETRAIN_H_ */
