/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>
#include <ctre/Phoenix.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <WPILib.h>

class Robot : public frc::IterativeRobot {
	XboxController Driver;
		XboxController Manipulator;
	WPI_TalonSRX LeftDrive;
		WPI_TalonSRX RightDrive;
		WPI_VictorSPX LeftDrive2;
		WPI_VictorSPX RightDrive1;
		WPI_VictorSPX RightDrive2;
		WPI_VictorSPX LeftDrive1;
public:
		Robot(void):


		Driver (0),
		Manipulator (1),
		LeftDrive(3),
		RightDrive (2),
		RightDrive1(6),
		RightDrive2(7),
		LeftDrive1(9),
		LeftDrive2(8)
	{}
	void RobotInit() {
		LeftDrive.ConfigSelectedFeedbackSensor(QuadEncoder,0,10);
				LeftDrive.SetSensorPhase(true);
				LeftDrive.SetNeutralMode(NeutralMode::Coast);
				LeftDrive.SetInverted(false);

				LeftDrive1.SetInverted(LeftDrive.GetInverted());
				LeftDrive2.SetInverted(LeftDrive.GetInverted());

				RightDrive.ConfigSelectedFeedbackSensor(QuadEncoder,0,10);
				RightDrive.SetSensorPhase(true);
				RightDrive.SetNeutralMode(NeutralMode::Coast);


				RightDrive1.SetInverted(RightDrive.GetInverted());
				RightDrive2.SetInverted(RightDrive.GetInverted());

		m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
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

	void AutonomousPeriodic() {
		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	void TeleopInit() {}

	void TeleopPeriodic() {
		LeftDrive1.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
		LeftDrive2.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
		RightDrive1.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());
		RightDrive2.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());
		LeftDrive.Set(ControlMode::PercentOutput,Driver.GetY(XboxController::kLeftHand));
		RightDrive.Set(ControlMode::PercentOutput,Driver.GetY(XboxController::kRightHand));
	}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
