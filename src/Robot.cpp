/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <ctre/Phoenix.h>
#include <iostream>
#include <WPILib.h>
#include <SmartDashboard/SmartDashboard.h>


void Robot::RobotInit() {
	double P=.11;
	double I=.01;
			LeftDrive.Config_kF(0,.5,10);
			RightDrive.Config_kF(0,.5,10);
			LeftDrive.EnableCurrentLimit(true);
			LeftDrive.ConfigOpenloopRamp(.125,10);
			LeftDrive.ConfigPeakCurrentLimit(30,10);
			LeftDrive.ConfigPeakCurrentDuration(2000,10);
			LeftDrive.Config_kP(0,P,10);
			LeftDrive.Config_kI(0,I,10);
			LeftDrive.Config_kD(0,0,10);
			LeftDrive.Config_IntegralZone(0,100,10);
			LeftDrive.ConfigSelectedFeedbackSensor(QuadEncoder,0,10);
			LeftDrive.SetSensorPhase(true);
			LeftDrive.SetNeutralMode(NeutralMode::Coast);
			LeftDrive.SetInverted(true);
			LeftDrive.ConfigMotionProfileTrajectoryPeriod(10,10);
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
			RightDrive.Config_kD(0,0,10);
			RightDrive.Config_IntegralZone(0,100,10);
			RightDrive.ConfigSelectedFeedbackSensor(QuadEncoder,0,10);
			RightDrive.SetSensorPhase(true);
			RightDrive.SetNeutralMode(NeutralMode::Coast);
			RightDrive.ConfigMotionProfileTrajectoryPeriod(10,10);
		//	RightDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic,
			//10, Constants::kTimeoutMs);

	m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
	m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

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
	// 		"Auto Selector", kAutoNameDefault);
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

void Robot::TeleopPeriodic() {

	RightMP.control();
	RightMP.PeriodicTask();
	LeftMP.control();
	LeftMP.PeriodicTask();
	if(Driver.GetAButton()== 1  ){

		SetValueMotionProfile setOutputL = LeftMP.getSetValue();
		SetValueMotionProfile setOutputR = RightMP.getSetValue();
		LeftDrive.Set(ControlMode::MotionProfile, setOutputL);
		RightDrive.Set(ControlMode::MotionProfile, setOutputR);

if(Driver.GetBButton()==1){

			RightMP.start();
			LeftMP.start();
}
	}else{
	LeftDrive.Set(ControlMode::PercentOutput,Driver.GetY(XboxController::kLeftHand));
	RightDrive.Set(ControlMode::PercentOutput,Driver.GetY(XboxController::kRightHand));
	LeftMP.reset();
	RightMP.reset();
	}


	LeftDrive1.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
	LeftDrive2.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
	RightDrive1.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());
	RightDrive2.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());
}

void Robot::TestPeriodic() {}

START_ROBOT_CLASS(Robot)
