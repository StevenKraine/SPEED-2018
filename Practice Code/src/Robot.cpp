/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//Right side, scale if ours we do the scale if not, we do a nice drive forward, if switch but not scale we do switch.
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
#include <DriveTrain.h>
#include <MotionProfile1.h>
#include <opencv2/imgproc/imgproc.hpp> /*Not Previously there*/
#include <opencv2/core/core.hpp> /*Not Previously there*/








class Robot : public frc::IterativeRobot{
	std::ofstream Record;
	std::ofstream RecordR;
	WPI_TalonSRX LeftDrive;
	WPI_TalonSRX Lift;
	WPI_VictorSPX LeftDrive1;
	WPI_VictorSPX LeftDrive2;
	WPI_VictorSPX RightDrive2;
	WPI_TalonSRX RightDrive;
	WPI_VictorSPX Climb1;
	WPI_VictorSPX RightDrive1;
	WPI_VictorSPX Lift1;
	WPI_VictorSPX IntakeLeft;
	WPI_VictorSPX IntakeRight;
	WPI_VictorSPX Climb3;
	WPI_VictorSPX Climb4;
	ADXRS450_Gyro Gyro;
	WPI_VictorSPX Climb2;
	XboxController Driver;
	XboxController Manipulator;
	Compressor Comp;
	DoubleSolenoid SpeedB;
	Solenoid Climb;
	Solenoid ClimbRelease;
	drivetrain drivetrain1;


public:



	int Row = 0;
	int CamFinder = 0;
	int CamFind = 0;
	bool CamSeek = 0;
	bool Middle = 0;
	bool LOW =0;
	bool up = 0;
	bool Intakeroo = 0;
	bool Intakeroo2 = 0;
	int IntakeStatus = 0;
	bool IntakeOveride = 1;
	double IntakeSpeed= 0;
	int AutoStatus = 70;//////////////////MAKE A REALLY BIG NUMBER AGAIN!!!!!!!!!////////////////
	int LiftPosition = 0;
	int LoopCount =0;
	double Seconds = 0;
	int loopcount = 0;
	bool Middy = 0;
	double EncoderDifference = 0;
	double GyroCorrection = Gyro.GetAngle();
	double Distance = 0;
	bool Recording = 0;
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
	int AutonomousChooser = 0;
	bool Reverse = 0;
	int testing=70;
	int LIF = 0;
	bool LIFTOVERIDE = 0;
	//bool Banner = banner.Get() ;
Robot(void):

LeftDrive(3),//3
	RightDrive (2),
	RightDrive1(6),
	RightDrive2(7),
	LeftDrive1(9),//9//
	LeftDrive2(8),
	Lift (1),
	Lift1 (11),
	IntakeLeft(5),
	IntakeRight(4),
	Driver (0),
	Manipulator(1),
	Comp(0),
	SpeedB(0,1,2),
	Climb(0,3),
	Climb3 (37),
	Climb1 (10),
	Climb2 (12),
	Climb4 (38),
	ClimbRelease(0,0),
	drivetrain1()






	{}
void RobotInit(){
		drivetrain1.DriveTrainInit();

		CameraServer::GetInstance()->StartAutomaticCapture();
		CameraServer::GetInstance()->StartAutomaticCapture();



		Climb1.SetInverted(false);
		Climb2.SetInverted(Climb1.GetInverted());
		Climb3.SetInverted(Climb1.GetInverted());
		Climb4.SetInverted(Climb1.GetInverted());

		Climb1.ConfigOpenloopRamp(.35,10);
			IntakeRight.SetInverted(true);
			IntakeLeft.SetInverted(true);



			Lift.SetInverted(true);
			Lift.ConfigSelectedFeedbackSensor(QuadEncoder,0,10);
			Lift.SetSensorPhase(false);
			Lift.Config_kP(0,.3,10);
			Lift.Config_kI(0,.00225,10);
			Lift.Config_kD(0,20,10);
			Lift.Config_IntegralZone(0,100,10);
			Lift.EnableCurrentLimit(true);
			Lift.ConfigPeakCurrentLimit(20,10);
			Lift.ConfigPeakCurrentDuration(10000,10);
			Lift.ConfigOpenloopRamp(.25,10);
			Lift.ConfigClosedloopRamp(0.25,10);
			Lift.SetNeutralMode(NeutralMode::Coast);
			Lift1.SetNeutralMode(NeutralMode::Coast);
			Lift1.SetInverted(!Lift.GetInverted());



			m_chooser.AddDefault(kRight, kRight);
			m_chooser.AddObject(kLeft, kLeft);
			m_chooser.AddObject(kCenter, kCenter);
			m_chooser.AddObject(kStraight, kStraight);
			m_chooser.AddObject(kLScale, kLScale);
			m_chooser.AddObject(kRScale, kRScale);
			m_chooser.AddObject(kOURSIDEL, kOURSIDEL);
			frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
		}
void OURSIDELInit(){std::string gameData;
gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
Gyro.Reset();
if(gameData[0]=='L' and gameData[1]=='L'){
	AutonomousChooser = 1;
	testing = 0;
}else if (gameData[0]=='R' and gameData[1]=='R'){
	AutonomousChooser = 20;
	testing = 0;
}else if(gameData[0]=='R' and gameData[1]=='L'){
	AutonomousChooser = 30;
	testing = 0;
}else if(gameData[0]=='L' and gameData[1]=='R'){
	AutonomousChooser = 40;
	testing = 0;
}}
void OURSIDEL (){
	switch(AutonomousChooser){
				case(1):
		LTWOCUBESCALESAMESIDE();
					break;
				case(20):
				StraightD();
					break;
				case(30):
		LTWOCUBESCALESAMESIDE();
					break;
				case(40):
		LSAMESIDESWITCHPLUSAPICKUP();
					break;
		}
}
void StraightD(){
	if(LeftDrive.GetSelectedSensorPosition(0)>-10000 and LeftDrive.GetSelectedSensorPosition(0)<-2000){
						SpeedB.Set(DoubleSolenoid::kForward);
					}else{
						SpeedB.Set(DoubleSolenoid::kReverse);
					}
				drivetrain1.DriveStraight(150,.5);
}
void CL(){



							 switch(testing){

							 	 case(0):


												 								 		if(LeftDrive.GetSelectedSensorPosition(0)>=-3000  ){
												 								 						SpeedB.Set(DoubleSolenoid::kForward);
												 								 					}else{SpeedB.Set(DoubleSolenoid::kReverse);}

												drivetrain1.DriveStraight(126,.5);
												 								 		IntakeSpeed = -.1;
												 								 		if(drivetrain1.InPosition(2) or loopcount>100){
												 								 			drivetrain1.reset();
												 								 			testing = 1;
												 								 			loopcount =0;

										 						break;
							 case(1):

							drivetrain1.DriveTurn(50,.5);
							if(drivetrain1.InPosition(2)==1 or loopcount>50){
								testing = 2;
								drivetrain1.reset();
								loopcount = 0;
							}

										 								 	break;
							 case(2):
								drivetrain1.DriveStraight(15,.65);
		if(drivetrain1.InPosition(6)==1 or loopcount>40){
			testing = 3;
		}
							break;
							 case(3):
									 drivetrain1.STOP();


											IntakeSpeed = 1;
											if(loopcount>25){

												drivetrain1.reset();
												testing = 4;
												loopcount = 0;
											}
							break;
							 case(4):
									 IntakeSpeed = 0;
		drivetrain1.DriveStraight(-70,.5);
				if(drivetrain1.InPosition(2)==1 or loopcount>50){
					drivetrain1.reset();
					testing = 5;
					loopcount = 0;
										 	}
				break;
							 case(5):
								 IntakeSpeed = 0;
										drivetrain1.DriveTurn(50,.5);
												if(drivetrain1.InPosition(6)==1 or loopcount>25){
													drivetrain1.reset();

													testing = 6;
													loopcount = 0;
																		 	}
												break;
							 case(6):
								 IntakeSpeed = -.75;
										drivetrain1.DriveStraight(47,.5);
										SpeedB.Set(DoubleSolenoid::kForward);
												if(drivetrain1.InPosition(2)==1 or loopcount>50){
													drivetrain1.reset();

													testing = 7;
													loopcount = 0;
																		 	}
												break;

							 case(7):
															 IntakeSpeed = -.2;
																	drivetrain1.DriveStraight(-47,.5);
																	SpeedB.Set(DoubleSolenoid::kReverse);
																			if(drivetrain1.InPosition(2)==1 or loopcount>50){
																				testing = 8;
																				drivetrain1.reset();
																				loopcount = 0;
																									 	}
																			break;
							 case(8):
															 IntakeSpeed = -.2;
														 drivetrain1.DriveTurn(-45,.65);//50
														 SpeedB.Set(DoubleSolenoid::kReverse);
														 if(drivetrain1.InPosition(6)==1 or loopcount>30){
															 testing = 9;
															 drivetrain1.reset();
															 loopcount = 0;
														 }
														 break;
							 case(9):
																						 IntakeSpeed = -.1;
							 Lift.Set(ControlMode::Position,3000);
																					 drivetrain1.DriveStraight(70,.5);
																					 SpeedB.Set(DoubleSolenoid::kReverse);
																					 if(drivetrain1.InPosition(2)==1 or loopcount>50){
																						 testing = 10;
																						 drivetrain1.reset();
																						 loopcount = 0;
																					 }
																					 break;
							 case(10):
																						 IntakeSpeed = 1;
																					 drivetrain1.STOP();
																					 SpeedB.Set(DoubleSolenoid::kReverse);
																					 if(drivetrain1.InPosition(2)==1 or loopcount>50){
																						 testing = 12;
																						 drivetrain1.reset();
																						 loopcount = 0;
																					 }
																					 break;

							 }

		}
}
void CR(){

										 		 switch(testing){
										 		 case(0):
		IntakeSpeed = -.1;

										 				drivetrain1.DriveTurn(50,.70);
										 								 if(drivetrain1.InPosition(2)==1 or loopcount>100){
										 									testing = 1;
										 									drivetrain1.reset();
										 									loopcount = 0;
										 									break;
										 		 case(1):



										 		 if(LeftDrive.GetSelectedSensorPosition(0)>=-3000 ){
										 			 SpeedB.Set(DoubleSolenoid::kForward);
										 		 }else{SpeedB.Set(DoubleSolenoid::kReverse);}

										 		 drivetrain1.DriveStraight(92,.5);

										 		if(drivetrain1.InPosition(2)==1 or loopcount>100){
										 			testing = 2;
										 			drivetrain1.reset();
										 			loopcount= 0;

										 		}

										 		 break;
										 		 case(2):
										 				 drivetrain1.STOP();
										 				 IntakeSpeed=1;

										 									if(loopcount>25){
										 										drivetrain1.reset();

												testing = 4;
												loopcount = 0;
											}
							break;
							 case(4):
									 IntakeSpeed = 0;
		drivetrain1.DriveStraight(-70,.5);
				if(drivetrain1.InPosition(2)==1 or loopcount>50){
					testing = 5;
					drivetrain1.reset();
					loopcount = 0;
										 	}
				break;
							 case(5):
								 IntakeSpeed = 0;
										drivetrain1.DriveTurn(-50,.5);
												if(drivetrain1.InPosition(6)==1 or loopcount>50){
													testing = 6;
													drivetrain1.reset();
													loopcount = 0;
																		 	}
												break;
							 case(6):
								 IntakeSpeed = -.75;
										drivetrain1.DriveStraight(48,.5);
										SpeedB.Set(DoubleSolenoid::kForward);
												if(drivetrain1.InPosition(2)==1 or loopcount>50){
													testing = 7;
													drivetrain1.reset();
													loopcount = 0;
																		 	}
												break;

							 case(7):
															 IntakeSpeed = -.2;
																	drivetrain1.DriveStraight(-48,.5);
																	SpeedB.Set(DoubleSolenoid::kReverse);
																			if(drivetrain1.InPosition(2)==1 or loopcount>50){
																				testing = 8;
																				drivetrain1.reset();
																				loopcount = 0;
																									 	}
																			break;
							 case(8):
															 IntakeSpeed = -.2;
														 drivetrain1.DriveTurn(50,.5);
														 SpeedB.Set(DoubleSolenoid::kReverse);
														 if(drivetrain1.InPosition(6)==1 or loopcount>50){
															 testing = 9;
															 drivetrain1.reset();
															 loopcount = 0;
														 }
														 break;
							 case(9):
																						 IntakeSpeed = -.1;
							 Lift.Set(ControlMode::Position,3000);
																					 drivetrain1.DriveStraight(70,.5);
																					 SpeedB.Set(DoubleSolenoid::kReverse);
																					 if(drivetrain1.InPosition(2)==1 or loopcount>50){
																						 testing = 10;
																						 drivetrain1.reset();
																						 loopcount = 0;
																					 }
																					 break;
							 case(10):
																						 IntakeSpeed = 1;
																					 drivetrain1.STOP();
																					 SpeedB.Set(DoubleSolenoid::kReverse);
																					 if(drivetrain1.InPosition(2)==1 or loopcount>50){
																						 testing = 12;
																						 drivetrain1.reset();
																						 loopcount = 0;
																					 }
																					 break;
										 		 }

		}
}
void RFARSIDESCALE(){switch(testing){
		case(0):
		testing = 1;
		drivetrain1.reset();
		Gyro.Reset();
		break;
		case(1):

				if(LeftDrive.GetSelectedSensorPosition(0)>-10000 and LeftDrive.GetSelectedSensorPosition(0)<-2000){
					SpeedB.Set(DoubleSolenoid::kForward);
				}else{
					SpeedB.Set(DoubleSolenoid::kReverse);
				}
			drivetrain1.DriveStraight(202,.5);
		if(drivetrain1.InPosition(2)==1 and LeftDrive.GetSelectedSensorVelocity(0)<=0 ){
			testing = 2;
			drivetrain1.reset();
			Gyro.Reset();
			loopcount = 0;
		}
		break;
// turn left
			case(2):

							drivetrain1.DriveTurn(-85,.5);//91
							if((drivetrain1.InPosition(2)==1  and LeftDrive.GetSelectedSensorVelocity(0)<=0 )){
								testing 	= 3;
								drivetrain1.reset();

			}

			break;
		case(3):

			drivetrain1.DriveStraight(205,.5);
		if((drivetrain1.InPosition(2)==1 and LeftDrive.GetSelectedSensorVelocity(0)<=0) ){
			testing = 4;
			drivetrain1.reset();
			Gyro.Reset();
			loopcount = 0;
		}
		break;
		case(4):

				drivetrain1.DriveTurn(110,.5);
		Lift.Set(ControlMode::Position,12000);
		IntakeSpeed = -.1;
		if(drivetrain1.InPosition(3)==1 ){
			testing = 5;
			drivetrain1.reset();
			loopcount = 0;
		}
		break;
		case(5):

		Lift.Set(ControlMode::Position,0);
		drivetrain1.DriveStraight(25,.5);
		if(drivetrain1.InPosition(2)==1){
			testing = 6;
			drivetrain1.reset();
			Seconds = 0;
			loopcount = 0;
			loopcount = 0;
			loopcount = 0;
			loopcount = 0;
		}
		break;
			case(6):
		Lift.Set(ControlMode::Position,19000 );
		Lift.Set(ControlMode::Position,19000 );
		drivetrain1.STOP();
		Lift.Set(ControlMode::Position,19000 );
		if(Lift.GetSelectedSensorPosition(0)>16000 ){
			IntakeSpeed = .75;
		}else{
			IntakeSpeed = -.1;
		}
		if(Lift.GetSelectedSensorPosition(0)>16000 and IntakeSpeed ==.75 and loopcount ==50){
							testing = 7;
							drivetrain1.reset();
							Gyro.Reset();
						}
		break;
			case(7):
		LeftDrive.ConfigPeakOutputForward(.5,10);
					 					 				RightDrive.ConfigPeakOutputForward(.85,10);
					 					 				LeftDrive.ConfigPeakOutputReverse(-.5,10);
					 					 						RightDrive.ConfigPeakOutputReverse(-.85,10);
					IntakeSpeed = 0;
					Lift.Set(ControlMode::Position,0);
					drivetrain1.DriveTurn(-120,.5);
					if(drivetrain1.InPosition(2)==1){
						testing = 8;
						drivetrain1.reset();
					}
					break;
			case(8):
		LeftDrive.ConfigPeakOutputForward(.75,10);
			 					 				RightDrive.ConfigPeakOutputForward(.85,10);
			 					 				LeftDrive.ConfigPeakOutputReverse(-.75,10);
			 					 						RightDrive.ConfigPeakOutputReverse(-.85,10);
					IntakeSpeed = -.5;
							Lift.Set(ControlMode::Position,0);
							drivetrain1.DriveStraight(44,.5);
							SpeedB.Set(DoubleSolenoid::kForward);
							if(drivetrain1.InPosition(2)==1){
								testing = 12;
								drivetrain1.reset();
							}
							break;
			case(9):
		LeftDrive.Set(ControlMode::PercentOutput,0);
							RightDrive.Set(ControlMode::PercentOutput,0);
							SpeedB.Set(DoubleSolenoid::kReverse);
							IntakeSpeed = -.1;
		break;
			case(10):
								IntakeSpeed = -.5;
										Lift.Set(ControlMode::Position,0);
										drivetrain1.DriveStraight(36,.5);
										SpeedB.Set(DoubleSolenoid::kForward);
										if(drivetrain1.InPosition(2)==1){
											testing = 9;
											drivetrain1.reset();
										}
										break;}}
void RTWOCUBESWITCHANDSCALESAMESIDE(){switch(testing){
		case(0):
		testing = 1;
		drivetrain1.reset();
		Gyro.Reset();
		loopcount = 0;
		break;
		case(1):

				if(LeftDrive.GetSelectedSensorPosition(0)>-12000 and LeftDrive.GetSelectedSensorPosition(0)<-1000){
					SpeedB.Set(DoubleSolenoid::kForward);
				}else{
					SpeedB.Set(DoubleSolenoid::kReverse);
				}
			drivetrain1.DriveStraight(233,.75);
		if((drivetrain1.InPosition(4)==1 and LeftDrive.GetSelectedSensorVelocity(0)==0) or loopcount >=80 ){
			testing = 2;
			drivetrain1.reset();
			Gyro.Reset();
			loopcount = 0;
		}

		break;

			case(2):

							drivetrain1.DriveTurn(-40,.5);
							if((drivetrain1.InPosition(3)==1 and LeftDrive.GetSelectedSensorVelocity(0)<=0)or loopcount >= 50){
								testing 	= 3;
								drivetrain1.reset();
								loopcount = 0;
			}

			break;
		case(3):
		Lift.Set(ControlMode::Position,19000);
					if(Lift.GetSelectedSensorPosition(0)>=18200){
						IntakeSpeed = 1;
					}else{
						IntakeSpeed = -.1;
					}
					LeftDrive.Set(ControlMode::PercentOutput,0);
					RightDrive.Set(ControlMode::PercentOutput,0);
		if( loopcount >=50 and Lift.GetSelectedSensorPosition(0)>=18400){
			testing = 4;
			drivetrain1.reset();
			Gyro.Reset();
			loopcount = 0;

		}
		break;
		case(4):
		Lift.Set(ControlMode::Position,0);
				drivetrain1.DriveTurn(-107,.5);

		IntakeSpeed = .75;
		if(drivetrain1.InPosition(2)==1 or loopcount >= 40){
			testing = 5;
			drivetrain1.reset();
			loopcount = 0;
			loopcount = 0;
		}
		break;
		case(5):

		Lift.Set(ControlMode::Position,000);
		drivetrain1.DriveStraight(57,.35);
		SpeedB.Set(DoubleSolenoid::kForward);
		IntakeSpeed = -.75;
		if(drivetrain1.InPosition(3)==1 or loopcount >= 55){
			testing = 6;
			drivetrain1.reset();
			Seconds = 0;
			loopcount = 0;
			loopcount = 0;
			loopcount = 0;
			loopcount = 0;
		}
		break;
			case(6):

		drivetrain1.DriveStraight(-6,.5);
		Lift.Set(ControlMode::Position,0 );
		IntakeSpeed=-.2;
		SpeedB.Set(DoubleSolenoid::kReverse);
		if(drivetrain1.InPosition(2)==1 or loopcount>=20){
							testing = 7;
							drivetrain1.reset();
							Gyro.Reset();
						}
		break;
			case(7):
		SpeedB.Set(DoubleSolenoid::kReverse);
					IntakeSpeed = -.10;
					Lift.Set(ControlMode::Position,4000);
					drivetrain1.DriveStraight(18,.5);
					if(drivetrain1.InPosition(2)==1 or loopcount >= 70){
						testing = 8;
						drivetrain1.reset();
						loopcount = 0;
					}
					break;
			case(8):

												Lift.Set(ControlMode::Position,4000);
							if(Lift.GetSelectedSensorPosition(0)>=3000){
								IntakeSpeed = 1;
							}else{
								IntakeSpeed = -.1;
							}
							LeftDrive.Set(ControlMode::PercentOutput,0);
												RightDrive.Set(ControlMode::PercentOutput,0);
							SpeedB.Set(DoubleSolenoid::kReverse);
							if( loopcount >=50){
								testing = 12;
								drivetrain1.reset();
								loopcount = 0;
							}
							break;
			case(9):
		LeftDrive.Set(ControlMode::PercentOutput,0);
							RightDrive.Set(ControlMode::PercentOutput,0);
							SpeedB.Set(DoubleSolenoid::kForward);
							IntakeSpeed = -.5;
							if(loopcount >= 20){
								testing = 10;
								SpeedB.Set(DoubleSolenoid::kReverse);
								drivetrain1.reset();
							}
		break;
			case(10):
								IntakeSpeed = -.15;
										Lift.Set(ControlMode::Position,0);
										drivetrain1.DriveTurn(-40,.5);
										SpeedB.Set(DoubleSolenoid::kReverse);
										if(drivetrain1.InPosition(2)==1){
											testing = 11;
											drivetrain1.reset();
										}
										break;
			case(11):
								IntakeSpeed = -.1;
										Lift.Set(ControlMode::Position,0);
										drivetrain1.DriveStraight(200,.5);
										SpeedB.Set(DoubleSolenoid::kReverse);
										if(drivetrain1.InPosition(2)==1){
											testing = 12;
											drivetrain1.reset();
										}
										break;
			case(12):
								IntakeSpeed = 0;
			LeftDrive.Set(ControlMode::PercentOutput,0);
										RightDrive.Set(ControlMode::PercentOutput,0);
									//	Lift.Set(ControlMode::Position,0);
									//	drivetrain1.DriveStraight(36);
									//	SpeedB.Set(DoubleSolenoid::kForward);
										/*	if(drivetrain1.InPosition(2)==1){
											testing = 9;
											drivetrain1.reset();
										}*/
										break;

		}}
void RTWOCUBESCALESAMESIDE(){switch(testing){
		case(0):
		testing = 1;
		drivetrain1.reset();
		Gyro.Reset();
		loopcount = 0;
		break;
		case(1):

				if(LeftDrive.GetSelectedSensorPosition(0)>-12000 and LeftDrive.GetSelectedSensorPosition(0)<-1000){
					SpeedB.Set(DoubleSolenoid::kForward);
				}else{
					SpeedB.Set(DoubleSolenoid::kReverse);
				}
			drivetrain1.DriveStraight(238,.75);
		if((drivetrain1.InPosition(4)==1 and LeftDrive.GetSelectedSensorVelocity(0)==0) or loopcount >=80 ){
			testing = 2;
			drivetrain1.reset();
			drivetrain1.reset();
			Gyro.Reset();
			loopcount = 0;
			loopcount = 0;
			loopcount = 0;
		}

		break;

			case(2):

							drivetrain1.DriveTurn(-38,.5);
							if((drivetrain1.InPosition(2)==1 and LeftDrive.GetSelectedSensorVelocity(0)<=0)or loopcount >= 50){
								testing 	= 3;
								drivetrain1.reset();
								drivetrain1.STOP();
								loopcount = 0;
			}

			break;
		case(3):
		Lift.Set(ControlMode::Position,19000);
					if(Lift.GetSelectedSensorPosition(0)>=18500){
						IntakeSpeed = 1;
					}else{
						IntakeSpeed = -.1;
					}
					drivetrain1.STOP();
		if( loopcount >=50 and Lift.GetSelectedSensorPosition(0)>=18000){
			testing = 4;
			drivetrain1.reset();
			Gyro.Reset();
			loopcount = 0;

		}
		break;
		case(4):
		Lift.Set(ControlMode::Position,0);
				drivetrain1.DriveTurn(-104,.5);

		IntakeSpeed = .75;
		if(drivetrain1.InPosition(2)==1 or loopcount >= 40){
			testing = 5;
			drivetrain1.reset();
			loopcount = 0;
			loopcount = 0;
		}
		break;
		case(5):

		Lift.Set(ControlMode::Position,000);
		drivetrain1.DriveStraight(62,.35);
		SpeedB.Set(DoubleSolenoid::kForward);
		IntakeSpeed = -.75;
		if(drivetrain1.InPosition(3)==1 or loopcount >= 55){
			testing = 6;
			drivetrain1.reset();
			Seconds = 0;
			loopcount = 0;
			loopcount = 0;
			loopcount = 0;
			loopcount = 0;
		}
		break;
			case(6):

		drivetrain1.DriveStraight(-45,.5);
		Lift.Set(ControlMode::Position,0 );
		IntakeSpeed=-.2;
		SpeedB.Set(DoubleSolenoid::kReverse);
		if(drivetrain1.InPosition(2)==1 or loopcount>=50){
							testing = 7;
							drivetrain1.reset();
							Gyro.Reset();
						}
		break;
			case(7):
		SpeedB.Set(DoubleSolenoid::kReverse);
					IntakeSpeed = -.10;
					Lift.Set(ControlMode::Position,0);
					drivetrain1.DriveTurn(84,.5);
					if(drivetrain1.InPosition(3)==1 or loopcount >= 70){
						testing = 8;
						drivetrain1.reset();
						loopcount = 0;
						loopcount = 0;
									loopcount = 0;
									loopcount = 0;
					}
					break;
			case(8):

												Lift.Set(ControlMode::Position,19000);
							if(Lift.GetSelectedSensorPosition(0)>=17000){
								IntakeSpeed = .75;
							}else{
								IntakeSpeed = -.1;
							}
							drivetrain1.STOP();
							SpeedB.Set(DoubleSolenoid::kReverse);
							if( loopcount >=200 and Lift.GetSelectedSensorPosition(0)>=18500){
								testing = 12;
								drivetrain1.reset();
								loopcount = 0;
							}
							break;
			case(9):
		LeftDrive.Set(ControlMode::PercentOutput,0);
							RightDrive.Set(ControlMode::PercentOutput,0);
							SpeedB.Set(DoubleSolenoid::kForward);
							IntakeSpeed = -.5;
							if(loopcount >= 20){
								testing = 10;
								SpeedB.Set(DoubleSolenoid::kReverse);
								drivetrain1.reset();
							}
		break;
			case(10):
								IntakeSpeed = -.15;
										Lift.Set(ControlMode::Position,0);
										drivetrain1.DriveTurn(-40,.5);
										SpeedB.Set(DoubleSolenoid::kReverse);
										if(drivetrain1.InPosition(2)==1){
											testing = 11;
											drivetrain1.reset();
										}
										break;
			case(11):
								IntakeSpeed = -.1;
										Lift.Set(ControlMode::Position,0);
										drivetrain1.DriveStraight(200,.5);
										SpeedB.Set(DoubleSolenoid::kReverse);
										if(drivetrain1.InPosition(2)==1){
											testing = 12;
											drivetrain1.reset();
										}
										break;
			case(12):
								IntakeSpeed = 0;
			drivetrain1.STOP();

										break;

		}}
void RSAMESIDESWITCHPLUSAPICKUP(){switch(testing){
		case(0):
		testing = 1;
		drivetrain1.reset();
		Gyro.Reset();
		break;
		case(1):

				if(LeftDrive.GetSelectedSensorPosition(0)>-10000 and LeftDrive.GetSelectedSensorPosition(0)<-2000){
					SpeedB.Set(DoubleSolenoid::kForward);
				}else{
					SpeedB.Set(DoubleSolenoid::kReverse);
				}
			drivetrain1.DriveStraight(140,.5);
		if(drivetrain1.InPosition(2)==1 and LeftDrive.GetSelectedSensorVelocity(0)<=0 ){
			testing = 2;
			drivetrain1.reset();
			Gyro.Reset();
			loopcount = 0;
		}
		break;

			case(2):

							drivetrain1.DriveTurn(-91,.5);
							if((drivetrain1.InPosition(2)==1  and LeftDrive.GetSelectedSensorVelocity(0)<=0)or loopcount>100){
								testing 	= 3;
								drivetrain1.reset();
								loopcount = 0;
			}

			break;
		case(3):
		Lift.Set(ControlMode::Position,4000);
					if(Lift.GetSelectedSensorPosition(0)>=3000){
						IntakeSpeed = .75;
					}else{
						IntakeSpeed = -.1;
					}
			drivetrain1.DriveStraight(20,.5);
		if(drivetrain1.InPosition(2)==1 or loopcount >=20){
			testing = 4;
			drivetrain1.reset();
			Gyro.Reset();

		}
		break;
		case(4):

				drivetrain1.DriveStraight(-20,.5);

		IntakeSpeed = -.1;
		if(drivetrain1.InPosition(3)==1 /*and LeftDrive.GetSelectedSensorVelocity(0)<=5*/){
			testing = 5;
			drivetrain1.reset();
		}
		break;
		case(5):

		Lift.Set(ControlMode::Position,000);
		drivetrain1.DriveTurn(89,.5);
		if(drivetrain1.InPosition(3)==1){
			testing = 6;
			drivetrain1.reset();
			Seconds = 0;
			loopcount = 0;
			loopcount = 0;
			loopcount = 0;
			loopcount = 0;
		}
		break;
			case(6):

		drivetrain1.STOP();
		Lift.Set(ControlMode::Position,0 );
		IntakeSpeed = 0;

		break;
			case(7):
		SpeedB.Set(DoubleSolenoid::kForward);
					IntakeSpeed = 0;
					Lift.Set(ControlMode::Position,0);
					drivetrain1.DriveTurn(-130,.5);
					if(drivetrain1.InPosition(3)==1){
						testing = 8;
						drivetrain1.reset();
						loopcount = 0;
					}
					break;
			case(8):
		LeftDrive.ConfigPeakOutputForward(.5,10);
			 					 				RightDrive.ConfigPeakOutputForward(.85,10);
			 					 				LeftDrive.ConfigPeakOutputReverse(-.5,10);
			 					 						RightDrive.ConfigPeakOutputReverse(-.85,10);
					IntakeSpeed = -.75;
							Lift.Set(ControlMode::Position,0);
							drivetrain1.DriveStraight(54,.5);
							SpeedB.Set(DoubleSolenoid::kForward);
							if(drivetrain1.InPosition(3)==1 or loopcount >=50){
								testing = 9;
								drivetrain1.reset();
								loopcount = 0;
							}
							break;
			case(9):
		drivetrain1.DriveStraight(-5,.5);
							SpeedB.Set(DoubleSolenoid::kForward);
							IntakeSpeed = -.5;
							if(loopcount >= 20){
								testing = 10;
								SpeedB.Set(DoubleSolenoid::kReverse);
								drivetrain1.reset();
							}
		break;
			case(10):
								IntakeSpeed = -.15;
										Lift.Set(ControlMode::Position,0);
										drivetrain1.DriveTurn(40,.5);
										SpeedB.Set(DoubleSolenoid::kReverse);
										if(drivetrain1.InPosition(2)==1){
											testing = 11;
											drivetrain1.reset();
										}
										break;
			case(11):
								IntakeSpeed = -.1;
										Lift.Set(ControlMode::Position,0);
										drivetrain1.DriveStraight(200,.5);
										SpeedB.Set(DoubleSolenoid::kReverse);
										if(drivetrain1.InPosition(2)==1){
											testing = 12;
											drivetrain1.reset();
										}
										break;
			case(12):
								IntakeSpeed = 0;
			LeftDrive.Set(ControlMode::PercentOutput,0);
										RightDrive.Set(ControlMode::PercentOutput,0);
									//	Lift.Set(ControlMode::Position,0);
									//	drivetrain1.DriveStraight(36);
									//	SpeedB.Set(DoubleSolenoid::kForward);
										/*	if(drivetrain1.InPosition(2)==1){
											testing = 9;
											drivetrain1.reset();
										}*/
										break;

		}}
/* This autonomous (along with the chooser code above) shows how to
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
void LScaleInit(){	std::string gameData;
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	Gyro.Reset();
	if(gameData[0]=='L' and gameData[1]=='L'){
		AutonomousChooser = 1;
		testing = 0;
	}else if (gameData[0]=='R' and gameData[1]=='R'){
		AutonomousChooser = 20;
		testing = 0;
	}else if(gameData[0]=='R' and gameData[1]=='L'){
		AutonomousChooser = 30;
		testing = 0;
	}else if(gameData[0]=='L' and gameData[1]=='R'){
		AutonomousChooser = 40;
		testing = 0;
	}}
void RScaleInit(){	std::string gameData;
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	Gyro.Reset();
	if(gameData[0]=='L' and gameData[1]=='L'){
		AutonomousChooser = 1;
		testing = 0;
	}else if (gameData[0]=='R' and gameData[1]=='R'){
		AutonomousChooser = 20;
		testing = 0;
	}else if(gameData[0]=='R' and gameData[1]=='L'){
		AutonomousChooser = 30;
		testing = 0;
	}else if(gameData[0]=='L' and gameData[1]=='R'){
		AutonomousChooser = 40;
		testing = 0;
	}}
void LeftInit(){
		std::string gameData;
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
 Gyro.Reset();
if(gameData[0]=='L' and gameData[1]=='L'){
		AutonomousChooser = 1;
		testing = 0;
	}else if (gameData[0]=='R' and gameData[1]=='R'){
		AutonomousChooser = 20;
		testing = 0;
	}else if(gameData[0]=='R' and gameData[1]=='L'){
		AutonomousChooser = 30;
		testing = 0;
	}else if(gameData[0]=='L' and gameData[1]=='R'){
		AutonomousChooser = 40;
		testing = 0;
	}}
void RightInit(){std::string gameData;
	 Gyro.Reset();
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

	if(gameData[0]=='R' and gameData[1]=='R'){
		AutonomousChooser = 1;
		testing = 0;
		}else if (gameData[0]=='L' and gameData[1]=='L'){
			AutonomousChooser = 20;
			testing = 0;
		}else if(gameData[0]=='L' and gameData[1]=='R'){
			AutonomousChooser = 30;
			testing = 0;
		}else if(gameData[0]=='R' and gameData[1]=='L'){
			AutonomousChooser = 40;
			testing = 0;
		}}
void CenterInit (){std::string gameData;
	 Gyro.Reset();
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	if(gameData[0]=='L' and gameData[1]=='L'){
		AutonomousChooser = 1;
		testing = 0;
		}else if (gameData[0]=='L' and gameData[1]=='R'){
			AutonomousChooser = 20;
			testing = 0;
		}else if(gameData[0]=='R' and gameData[1]=='R'){
			AutonomousChooser = 30;
			testing = 0;
		}else if(gameData[0]=='R' and gameData[1]=='L'){
			AutonomousChooser = 40;
			testing = 0;
		}}
void StraightInit (){std::string gameData;
	 Gyro.Reset();
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();


	if(gameData[0]=='L' and gameData[1]=='L'){
		AutonomousChooser = 1;
		testing = 0;
		}else if (gameData[0]=='L' and gameData[1]=='R'){
			AutonomousChooser = 20;
			testing = 0;
		}else if(gameData[0]=='R' and gameData[1]=='R'){
			AutonomousChooser = 30;
			testing = 0;
		}else if(gameData[0]=='R' and gameData[1]=='L'){
			AutonomousChooser = 40;
			testing = 0;
		}}
void AutonomousInit() override {
		RightDrive.SetSelectedSensorPosition(0,0,10);
		LeftDrive.SetSelectedSensorPosition(0,0,10);
		Gyro.Reset();
		//		Lift.SetSelectedSensorPosition(0,0,10);
		m_autoSelected = m_chooser.GetSelected();
		LoopCount = 0;
		Seconds = 0;
		AutoStatus = 0;
		// m_autoSelected = SmartDashboard::GetString(
		// 		"Auto Selector", kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;
		do  {
		if (m_autoSelected == kLeft) {
			LeftInit();
		} else if (m_autoSelected == kRight) {
			RightInit();
		}else if(m_autoSelected == kCenter){
			CenterInit();
		}else if(m_autoSelected == kStraight){
			StraightInit();
		}else if(m_autoSelected == kLScale){
			LScaleInit();
		}else if(m_autoSelected == kRScale){
			RScaleInit();
		}else if(m_autoSelected == kOURSIDEL){
			OURSIDELInit();
		}
		}while(testing == 70);
	}
void Left(){
		LeftDrive1.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
		LeftDrive2.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
		RightDrive1.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());
		RightDrive2.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());
		IntakeLeft.Set(ControlMode::PercentOutput,IntakeSpeed);
		IntakeRight.Set(ControlMode::PercentOutput,-1*IntakeSpeed);
		Lift1.Set(ControlMode::PercentOutput,Lift.GetMotorOutputPercent());
		SmartDashboard::PutNumber("AutoStatus", AutoStatus);
		SmartDashboard::PutNumber("velocity",LeftDrive.GetSelectedSensorVelocity(0));
		SmartDashboard::PutNumber("position",RightDrive.GetSelectedSensorPosition(0));
		 LeftRotations =LeftDrive.GetSelectedSensorPosition(0);

					 		 velocityL = LeftDrive.GetSelectedSensorVelocity(0);
					 		 velocity = RightDrive.GetSelectedSensorVelocity(0);
					 		 RightRotations = RightDrive.GetSelectedSensorPosition(0);
					 		VelL = (velocityL)/4096*600;
					 		VelR = (velocity)/4096*600;
					 		PositionR = (RightRotations/4096)*1.0;
					 		PositionL = (LeftRotations/4096)*1.0;

					 		VelocityCorrection = (velocityL-velocity)/5000;
					 		 PositionOffset = (LeftRotations-RightRotations)/1000;

			/*	 switch(AutoStatus){




			 case (20):
			 	LeftDrive.ConfigPeakOutputForward(.5,10);
			 				RightDrive.ConfigPeakOutputForward(.6,10);
			 				LeftDrive.ConfigPeakOutputReverse(-.5,10);
			 						RightDrive.ConfigPeakOutputReverse(-.6,10);
			 		if(LeftDrive.GetMotorOutputPercent()==0 and LeftDrive.GetSelectedSensorPosition(0)>-1000){
			 				RightDrive.Set(ControlMode::PercentOutput,-1);
			 			}else{
			 				RightDrive.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent() +PositionOffset+VelocityCorrection);
			 			}
			 		if(LeftDrive.GetSelectedSensorPosition(0)<=-60000 ){
			 			Lift.Set(ControlMode::Position,10000);
			 		}
			 		if(LeftDrive.GetSelectedSensorPosition(0)>=-3000 ){
			 						SpeedB.Set(DoubleSolenoid::kForward);
			 					}else{SpeedB.Set(DoubleSolenoid::kReverse);}

			 		LeftDrive.Set(ControlMode::Position,-82921);

			 		if(LeftDrive.GetSelectedSensorPosition(0)<-80000 and LeftDrive.GetSelectedSensorVelocity(0)==0){
			 			LeftDrive.SetSelectedSensorPosition(0,0,10);
			 			RightDrive.SetSelectedSensorPosition(0,0,10);
			 			AutoStatus = 1;
			 			loopcount =0;
			 		}

			 	break;
			 case(1):
			 	Lift.Set(ControlMode::Position,19000);
			 	LeftDrive.ConfigPeakOutputForward(.5,10);
			 				RightDrive.ConfigPeakOutputForward(.5,10);
			 				LeftDrive.ConfigPeakOutputReverse(-.5,10);
			 						RightDrive.ConfigPeakOutputReverse(-.5,10);
			 if(Lift.GetSelectedSensorPosition(0)>17000){
			 IntakeSpeed = .65;
			 }
			 	LeftDrive.Set(ControlMode::Position,-5000);
			 	RightDrive.Set(ControlMode::Position,5000);
			 	if(RightDrive.GetSelectedSensorPosition(0)>3750 and loopcount>50){

			 		LeftDrive.SetSelectedSensorPosition(0,0,10);
			 						RightDrive.SetSelectedSensorPosition(0,0,10);
			 						Lift.Set(ControlMode::Position,0);
			 						AutoStatus = 3;
			 	}
			 //Scale Our Side Done
			 	break;
			 case(3):

			 		LeftDrive.Set(ControlMode::Position,-8900);
			         RightDrive.Set(ControlMode::Position,8900);
			         if(RightDrive.GetSelectedSensorPosition(0)>6000 and LeftDrive.GetSelectedSensorPosition(0)<-6000  and LeftDrive.GetSelectedSensorVelocity(0)==00){
			         	LeftDrive.SetSelectedSensorPosition(0,0,10);
			         	SpeedB.Set(DoubleSolenoid::kForward);
			         	RightDrive.SetSelectedSensorPosition(0,0,10);
			         	AutoStatus = 4;}
			 		break;
			 case(4):
			 	SpeedB.Set(DoubleSolenoid::kForward);
			 	LeftDrive.ConfigPeakOutputForward(.5,10);
			 						RightDrive.ConfigPeakOutputForward(.6,10);
			 						LeftDrive.ConfigPeakOutputReverse(-.5,10);
			 								RightDrive.ConfigPeakOutputReverse(-.6,10);

			 						RightDrive.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent() +PositionOffset+VelocityCorrection);

			 				Lift.Set(ControlMode::Position,0);
			 				IntakeSpeed = -.75;

			 				LeftDrive.Set(ControlMode::Position,-20117);

			 if(RightDrive.GetSelectedSensorPosition(0)<-5000 and LeftDrive.GetSelectedSensorVelocity(0)== 0  ){
			 	LeftDrive.SetSelectedSensorPosition(0,0,10);
			 	RightDrive.SetSelectedSensorPosition(0,0,10);
			 	loopcount = 0;
			 	AutoStatus = 5;}
			 		break;

			 	case(5):
			 		SpeedB.Set(DoubleSolenoid::kReverse);
			 		Lift.Set(ControlMode::Position,3000);
			 						IntakeSpeed = -.1;
			 			LeftDrive.Set(ControlMode::PercentOutput,000);
			 	        RightDrive.Set(ControlMode::PercentOutput,000);
			 	        if(loopcount >= 20){
			 	        	LeftDrive.SetSelectedSensorPosition(0,0,10);
			 	        	SpeedB.Set(DoubleSolenoid::kForward);
			 	        	RightDrive.SetSelectedSensorPosition(0,0,10);
			 	        	AutoStatus = 6;}
			 			break;
			 	case(6):
			 				SpeedB.Set(DoubleSolenoid::kReverse);
			 				Lift.Set(ControlMode::Position,3000);
			 								IntakeSpeed = -.1;
			 								RightDrive.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent() +PositionOffset+VelocityCorrection);
			 								LeftDrive.Set(ControlMode::Position,-5500);
			 			        if(RightDrive.GetSelectedSensorPosition(0)<-2500 and LeftDrive.GetSelectedSensorPosition(0)<-2500  and LeftDrive.GetSelectedSensorVelocity(0)>=10){
			 			        	//LeftDrive.SetSelectedSensorPosition(0,0,10);
			 			        	SpeedB.Set(DoubleSolenoid::kForward);
			 			        	//RightDrive.SetSelectedSensorPosition(0,0,10);
			 			        	AutoStatus = 7;
			 			        loopcount = 0;}
			 					break;
			 	case(7):
			 				SpeedB.Set(DoubleSolenoid::kReverse);
			 				Lift.Set(ControlMode::Position,3000);
			 								IntakeSpeed = 1;
			 					LeftDrive.Set(ControlMode::PercentOutput,000);
			 			        RightDrive.Set(ControlMode::PercentOutput,000);
			 			        if(loopcount==20){
			 			        	LeftDrive.SetSelectedSensorPosition(0,0,10);
			 			        	SpeedB.Set(DoubleSolenoid::kForward);
			 			        	RightDrive.SetSelectedSensorPosition(0,0,10);
			 			        	AutoStatus = 8;}
			 					break;
			 	case(8):
			 				SpeedB.Set(DoubleSolenoid::kForward);
			 				Lift.Set(ControlMode::Position,0);
			 								IntakeSpeed = 0;
			 					LeftDrive.Set(ControlMode::Position,8000);
			 			        RightDrive.Set(ControlMode::Position,5000);
			 			        if(RightDrive.GetSelectedSensorPosition(0)>6000 and LeftDrive.GetSelectedSensorPosition(0)<-6000  and LeftDrive.GetSelectedSensorVelocity(0)==00){
			 			        //	LeftDrive.SetSelectedSensorPosition(0,0,10);
			 			        	SpeedB.Set(DoubleSolenoid::kForward);
			 			        	//RightDrive.SetSelectedSensorPosition(0,0,10);
			 			        	AutoStatus = 70;}
			 					break;}*/
		switch(AutonomousChooser){
		case(1):
		LTWOCUBESWITCHANDSCALESAMESIDE();
					break;
		case(20):
		LFARSIDESCALE();
					break;
		case(30):
		LTWOCUBESCALESAMESIDE();
			break;
		case(40):
		LSAMESIDESWITCHPLUSAPICKUP();
			break;
		}

}
void SingleScaleL(){switch(testing){
		case(0):
		testing = 1;
		drivetrain1.reset();
		Gyro.Reset();
		loopcount = 0;
		break;
		case(1):

				if(LeftDrive.GetSelectedSensorPosition(0)>-12000 and LeftDrive.GetSelectedSensorPosition(0)<-1000){
					SpeedB.Set(DoubleSolenoid::kForward);
				}else{
					SpeedB.Set(DoubleSolenoid::kReverse);
				}
			drivetrain1.DriveStraight(238,.75);
			IntakeSpeed = -.1;
		if((drivetrain1.InPosition(4)==1 and LeftDrive.GetSelectedSensorVelocity(0)==0) or loopcount >=80 ){
			testing = 2;
			drivetrain1.reset();
			drivetrain1.reset();
			Gyro.Reset();
			loopcount = 0;
			loopcount = 0;
			loopcount = 0;
		}

		break;

			case(2):
		IntakeSpeed = -.1;
							drivetrain1.DriveTurn(38,.5);//44
							if((drivetrain1.InPosition(2)==1 and LeftDrive.GetSelectedSensorVelocity(0)<=0)or loopcount >= 50){
								testing 	= 3;
								drivetrain1.reset();
								drivetrain1.STOP();
								loopcount = 0;
			}

			break;
		case(3):
		Lift.Set(ControlMode::Position,19000);
					if(Lift.GetSelectedSensorPosition(0)>=18000 ){
						IntakeSpeed = .75;
					}else{
						IntakeSpeed = -.1;

					}
					drivetrain1.STOP();
		if( loopcount >=50 and Lift.GetSelectedSensorPosition(0)>=18000){
			testing = 4;
			drivetrain1.reset();
			Gyro.Reset();
			loopcount = 0;

		}
		break;
		//turn for second cube
		case(4):
		Lift.Set(ControlMode::Position,0);
				drivetrain1.DriveStraight(-30,.5);//106

		IntakeSpeed = .75;
		if(drivetrain1.InPosition(2)==1 or loopcount >= 75){
			testing = 5;
			drivetrain1.reset();
			loopcount = 0;
			loopcount = 0;
		}
		break;
		case(5):
					drivetrain1.STOP()	;
				IntakeSpeed = 0;
				break;
					}

}
void SingleScaleR(){
	switch(testing){
		case(0):
		testing = 1;
		drivetrain1.reset();
		Gyro.Reset();
		loopcount = 0;
		break;
		case(1):

				if(LeftDrive.GetSelectedSensorPosition(0)>-12000 and LeftDrive.GetSelectedSensorPosition(0)<-1000){
					SpeedB.Set(DoubleSolenoid::kForward);
				}else{
					SpeedB.Set(DoubleSolenoid::kReverse);
				}
			drivetrain1.DriveStraight(238,.75);
			IntakeSpeed = -.1;
		if((drivetrain1.InPosition(4)==1 and LeftDrive.GetSelectedSensorVelocity(0)==0) or loopcount >=80 ){
			testing = 2;
			drivetrain1.reset();
			drivetrain1.reset();
			Gyro.Reset();
			loopcount = 0;
			loopcount = 0;
			loopcount = 0;
		}

		break;

			case(2):
		IntakeSpeed = -.1;
							drivetrain1.DriveTurn(-38,.5);//44
							if((drivetrain1.InPosition(2)==1 and LeftDrive.GetSelectedSensorVelocity(0)<=0)or loopcount >= 50){
								testing 	= 3;
								drivetrain1.reset();
								drivetrain1.STOP();
								loopcount = 0;
			}

			break;
		case(3):
		Lift.Set(ControlMode::Position,19000);
					if(Lift.GetSelectedSensorPosition(0)>=18000 ){
						IntakeSpeed = .75;
					}else{
						IntakeSpeed = -.1;

					}
					drivetrain1.STOP();
		if( loopcount >=50 and Lift.GetSelectedSensorPosition(0)>=18000){
			testing = 4;
			drivetrain1.reset();
			Gyro.Reset();
			loopcount = 0;

		}
		break;
		//turn for second cube
		case(4):
		Lift.Set(ControlMode::Position,0);
				drivetrain1.DriveStraight(-30,.5);//106

		IntakeSpeed = .75;
		if(drivetrain1.InPosition(2)==1 or loopcount >= 75){
			testing = 5;
			drivetrain1.reset();
			loopcount = 0;
			loopcount = 0;
		}
		break;
		case(5):
			drivetrain1.STOP()	;
		IntakeSpeed = 0;
		break;

		}
}
void LScale(){
	switch(AutonomousChooser){
			case(1):
		LTWOCUBESCALESAMESIDE();
				break;
			case(20):
		LFARSIDESCALE();
				break;
			case(30):
		LTWOCUBESCALESAMESIDE();
				break;
			case(40):
		LFARSIDESCALE();
				break;
	}
}
void RScale(){
	switch(AutonomousChooser){
			case(1):
			RFARSIDESCALE();
				break;
			case(20):
			RTWOCUBESCALESAMESIDE();
				break;
			case(30):
			RFARSIDESCALE();
				break;
			case(40):
		RTWOCUBESCALESAMESIDE();
				break;
	}
}
void Right(){
		switch(AutonomousChooser){
		case(1):
		SingleScaleR();
			break;
		case(20):
		StraightD();
			break;
		case(30):
		SingleScaleR();
			break;
		case(40):
		 RSAMESIDESWITCHPLUSAPICKUP();//Change this one
			break;
																	}

		LeftDrive1.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
				LeftDrive2.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
				RightDrive1.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());
				RightDrive2.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());
				IntakeLeft.Set(ControlMode::PercentOutput,IntakeSpeed);
				IntakeRight.Set(ControlMode::PercentOutput,-1*IntakeSpeed);
				Lift1.Set(ControlMode::PercentOutput,Lift.GetMotorOutputPercent());
				SmartDashboard::PutNumber("AutoStatus", AutoStatus);
				SmartDashboard::PutNumber("velocity",LeftDrive.GetSelectedSensorVelocity(0));
				SmartDashboard::PutNumber("position",RightDrive.GetSelectedSensorPosition(0));


						}
void Center (){

			LeftDrive1.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
			LeftDrive2.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
			RightDrive1.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());
			RightDrive2.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());
			IntakeLeft.Set(ControlMode::PercentOutput,IntakeSpeed);
			IntakeRight.Set(ControlMode::PercentOutput,-1*IntakeSpeed);
			Lift1.Set(ControlMode::PercentOutput,Lift.GetMotorOutputPercent());


			switch(AutonomousChooser){
									case(1):
										CL();
										break;
									case(20):
										CL();
										break;
									case(30):
										CR();
										break;
									case(40):
										CR();
										break;
									}}
void Straight(){

		LFARSIDESCALE();
}
void AutonomousPeriodic() {
Lift.ConfigPeakOutputReverse(-.75,0);
IntakeLeft.Set(ControlMode::PercentOutput,IntakeSpeed);
			IntakeRight.Set(ControlMode::PercentOutput,-1*IntakeSpeed);

		std::ofstream Record;
			Record.open("/home/lvuser/Record.txt", std::ofstream::out | std::ofstream::app);

			Record<<","<<AutoStatus<< LeftDrive.GetSelectedSensorPosition(0)<<","<<RightDrive.GetSelectedSensorPosition(0)<< endl;

			Record.close();

		EncoderDifference = (LeftDrive.GetSelectedSensorPosition(0)-RightDrive.GetSelectedSensorPosition(0))/10000;
		SmartDash();
		std::shared_ptr<NetworkTable> table =   NetworkTable::GetTable("limelight");
		float targetOffsetAngle_Horizontal = table->GetNumber("tx",0);
					float targetOffsetAngle_Vertical = table->GetNumber("ty",0);
					float targetArea = table->GetNumber("ta",0);
					float targetSkew = table->GetNumber("ts",0);


		GyroCorrection = Gyro.GetAngle()/1000;
		loopcount = loopcount +1;
		Seconds = LoopCount/50;
		Lift1.Set(ControlMode::PercentOutput,Lift.GetMotorOutputPercent());
		if (m_autoSelected == kLeft) {
		/*	LeftDrive1.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
					LeftDrive2.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent());
					RightDrive1.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());
					RightDrive2.Set(ControlMode::PercentOutput,RightDrive.GetMotorOutputPercent());
					IntakeLeft.Set(ControlMode::PercentOutput,IntakeSpeed);
					IntakeRight.Set(ControlMode::PercentOutput,-1*IntakeSpeed);
					Lift1.Set(ControlMode::PercentOutput,Lift.GetMotorOutputPercent());
					SmartDashboard::PutNumber("AutoStatus", AutoStatus);
					SmartDashboard::PutNumber("velocity",LeftDrive.GetSelectedSensorVelocity(0));
					SmartDashboard::PutNumber("position",RightDrive.GetSelectedSensorPosition(0));
					 LeftRotations =LeftDrive.GetSelectedSensorPosition(0);

								 		 velocityL = LeftDrive.GetSelectedSensorVelocity(0);
								 		 velocity = RightDrive.GetSelectedSensorVelocity(0);
								 		 RightRotations = RightDrive.GetSelectedSensorPosition(0);
								 		VelL = (velocityL)/4096*600;
								 		VelR = (velocity)/4096*600;
								 		PositionR = (RightRotations/4096)*1.0;
								 		PositionL = (LeftRotations/4096)*1.0;

								 		VelocityCorrection = (velocityL-velocity)/5000;
								 		 PositionOffset = (LeftRotations-RightRotations)/1000;

							 switch(AutoStatus){




						 case (0):
						 	LeftDrive.ConfigPeakOutputForward(.5,10);
						 				RightDrive.ConfigPeakOutputForward(.6,10);
						 				LeftDrive.ConfigPeakOutputReverse(-.5,10);
						 						RightDrive.ConfigPeakOutputReverse(-.6,10);
						 		if(LeftDrive.GetMotorOutputPercent()==0 and LeftDrive.GetSelectedSensorPosition(0)>-1000){
						 				RightDrive.Set(ControlMode::PercentOutput,-1);
						 			}else{
						 				RightDrive.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent() +PositionOffset+VelocityCorrection);
						 			}
						 		if(LeftDrive.GetSelectedSensorPosition(0)<=-60000 ){
						 			Lift.Set(ControlMode::Position,10000);
						 		}
						 		if(LeftDrive.GetSelectedSensorPosition(0)>=-3000 ){
						 						SpeedB.Set(DoubleSolenoid::kForward);
						 					}else{SpeedB.Set(DoubleSolenoid::kReverse);}

						 		LeftDrive.Set(ControlMode::Position,-82921);

						 		if(LeftDrive.GetSelectedSensorPosition(0)<-80000 and LeftDrive.GetSelectedSensorVelocity(0)==0){
						 			LeftDrive.SetSelectedSensorPosition(0,0,10);
						 			RightDrive.SetSelectedSensorPosition(0,0,10);
						 			AutoStatus = 1;
						 			loopcount =0;
						 		}

						 	break;
						 case(1):
						 	Lift.Set(ControlMode::Position,19000);
						 	LeftDrive.ConfigPeakOutputForward(.5,10);
						 				RightDrive.ConfigPeakOutputForward(.5,10);
						 				LeftDrive.ConfigPeakOutputReverse(-.5,10);
						 						RightDrive.ConfigPeakOutputReverse(-.5,10);
						 if(Lift.GetSelectedSensorPosition(0)>17000){
						 IntakeSpeed = .65;
						 }
						 	LeftDrive.Set(ControlMode::Position,-5000);
						 	RightDrive.Set(ControlMode::Position,5000);
						 	if(RightDrive.GetSelectedSensorPosition(0)>3750 and loopcount>50){

						 		LeftDrive.SetSelectedSensorPosition(0,0,10);
						 						RightDrive.SetSelectedSensorPosition(0,0,10);
						 						Lift.Set(ControlMode::Position,0);
						 						AutoStatus = 3;
						 	}
						 //Scale Our Side Done
						 	break;
						 case(3):

						 		LeftDrive.Set(ControlMode::Position,-8900);
						         RightDrive.Set(ControlMode::Position,8900);
						         if(RightDrive.GetSelectedSensorPosition(0)>6000 and LeftDrive.GetSelectedSensorPosition(0)<-6000  and LeftDrive.GetSelectedSensorVelocity(0)==00){
						         	LeftDrive.SetSelectedSensorPosition(0,0,10);
						         	SpeedB.Set(DoubleSolenoid::kForward);
						         	RightDrive.SetSelectedSensorPosition(0,0,10);
						         	AutoStatus = 4;}
						 		break;
						 case(4):
						 	SpeedB.Set(DoubleSolenoid::kForward);
						 	LeftDrive.ConfigPeakOutputForward(.5,10);
						 						RightDrive.ConfigPeakOutputForward(.6,10);
						 						LeftDrive.ConfigPeakOutputReverse(-.5,10);
						 								RightDrive.ConfigPeakOutputReverse(-.6,10);

						 						RightDrive.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent() +PositionOffset+VelocityCorrection);

						 				Lift.Set(ControlMode::Position,0);
						 				IntakeSpeed = -.75;

						 				LeftDrive.Set(ControlMode::Position,-20117);

						 if(RightDrive.GetSelectedSensorPosition(0)<-5000 and LeftDrive.GetSelectedSensorVelocity(0)== 0  ){
						 	LeftDrive.SetSelectedSensorPosition(0,0,10);
						 	RightDrive.SetSelectedSensorPosition(0,0,10);
						 	loopcount = 0;
						 	AutoStatus = 5;}
						 		break;

						 	case(5):
						 		SpeedB.Set(DoubleSolenoid::kReverse);
						 		Lift.Set(ControlMode::Position,3000);
						 						IntakeSpeed = -.1;
						 			LeftDrive.Set(ControlMode::PercentOutput,000);
						 	        RightDrive.Set(ControlMode::PercentOutput,000);
						 	        if(loopcount >= 20){
						 	        	LeftDrive.SetSelectedSensorPosition(0,0,10);
						 	        	SpeedB.Set(DoubleSolenoid::kForward);
						 	        	RightDrive.SetSelectedSensorPosition(0,0,10);
						 	        	AutoStatus = 6;}
						 			break;
						 	case(6):
						 				SpeedB.Set(DoubleSolenoid::kReverse);
						 				Lift.Set(ControlMode::Position,3000);
						 								IntakeSpeed = -.1;
						 								RightDrive.Set(ControlMode::PercentOutput,LeftDrive.GetMotorOutputPercent() +PositionOffset+VelocityCorrection);
						 								LeftDrive.Set(ControlMode::Position,-5500);
						 			        if(RightDrive.GetSelectedSensorPosition(0)<-2500 and LeftDrive.GetSelectedSensorPosition(0)<-2500  and LeftDrive.GetSelectedSensorVelocity(0)>=10){
						 			        	//LeftDrive.SetSelectedSensorPosition(0,0,10);
						 			        	SpeedB.Set(DoubleSolenoid::kForward);
						 			        	//RightDrive.SetSelectedSensorPosition(0,0,10);
						 			        	AutoStatus = 7;
						 			        loopcount = 0;}
						 					break;
						 	case(7):
						 				SpeedB.Set(DoubleSolenoid::kReverse);
						 				Lift.Set(ControlMode::Position,3000);
						 								IntakeSpeed = 1;
						 					LeftDrive.Set(ControlMode::PercentOutput,000);
						 			        RightDrive.Set(ControlMode::PercentOutput,000);
						 			        if(loopcount==20){
						 			        	LeftDrive.SetSelectedSensorPosition(0,0,10);
						 			        	SpeedB.Set(DoubleSolenoid::kForward);
						 			        	RightDrive.SetSelectedSensorPosition(0,0,10);
						 			        	AutoStatus = 8;}
						 					break;
						 	case(8):
						 				SpeedB.Set(DoubleSolenoid::kForward);
						 				Lift.Set(ControlMode::Position,0);
						 								IntakeSpeed = 0;
						 					LeftDrive.Set(ControlMode::Position,8000);
						 			        RightDrive.Set(ControlMode::Position,5000);
						 			        if(RightDrive.GetSelectedSensorPosition(0)>6000 and LeftDrive.GetSelectedSensorPosition(0)<-6000  and LeftDrive.GetSelectedSensorVelocity(0)==00){
						 			        //	LeftDrive.SetSelectedSensorPosition(0,0,10);
						 			        	SpeedB.Set(DoubleSolenoid::kForward);
						 			        	//RightDrive.SetSelectedSensorPosition(0,0,10);
						 			        	AutoStatus = 70;}
						 					break;}*/
			//LRL();
			Left();
		} else if (m_autoSelected == kRight)  {
			Right();
		}else if(m_autoSelected == kCenter){
			Center();
		}else if(m_autoSelected == kStraight){
			Straight();
		}else if(m_autoSelected == kLScale){
			LScale();
		}else if(m_autoSelected == kRScale){
			RScale();
		}else if(m_autoSelected == kOURSIDEL){
			OURSIDEL();
		}
	}
void LiftControl(){
SmartDashboard::PutNumber("OVERIDE",LIFTOVERIDE);
if(Manipulator.GetStickButtonPressed(XboxController::kLeftHand)==1){
	LIFTOVERIDE=!LIFTOVERIDE;
}

	Lift1.Set(ControlMode::PercentOutput,Lift.GetMotorOutputPercent());


	if(LIFTOVERIDE==0){
		if(SpeedB.Get()==2){
		Lift.Set(ControlMode::Position,LIF);
	}else{
		Lift.Set(ControlMode::Position,0);
	}

	if(Manipulator.GetAButtonPressed() == 1){
			Middy = !Middy;
			Middle = 0;
			Recording = 0;
			LOW= 0;
			LIF = 12000;
		}
		if(Manipulator.GetXButtonPressed() == 1){
			Middy = 0;
			Middle = !Middle;
			Recording = 0;
			LOW = 0;
			LIF = 15000;
		}
		if(Manipulator.GetYButtonPressed() == 1){
				Middy = 0;
				Middle = 0;
				Recording = !Recording;
				LOW = 0;
				LIF = 19000;
			}
		if(Manipulator.GetStickButtonPressed(XboxController::kRightHand) == 1){
						Middy = 0;
						Middle = 0;
						Recording = !Recording;
						LOW = !LOW;
						LIF = 3000;
					}
		SmartDashboard::PutNumber("Trigger",Manipulator.GetTriggerAxis(XboxController::kRightHand));
		if(Manipulator.GetTriggerAxis(XboxController::kRightHand)>.5){
			LIF = LIF-1000;
		}

		if (Lift.GetSelectedSensorPosition(0)>200 and Recording != 1 and Middle !=1 and Middy !=1 and LOW!=1){
			LIF= 0;
		}else if(Lift.GetSelectedSensorPosition(0)<200 and Recording != 1 and Middle !=1 and Middy !=1 and LOW!=1){
			LIF = 0;
		}
		if(Lift.GetSelectedSensorPosition(0)>4000){

			Lift.ConfigPeakOutputReverse(-.65,10);
		}else if(Lift.GetSelectedSensorPosition(0)>4000){
				Lift.ConfigPeakOutputReverse(-.25,10);
			}else if(Lift.GetSelectedSensorPosition(0)>2000){
				Lift.ConfigPeakOutputReverse(-.1,10);

	 }}else{
		 if(Manipulator.GetStickButtonPressed(XboxController::kRightHand)){
			 Lift.SetSelectedSensorPosition(0,0,10);
		 }
		 Lift.ConfigPeakOutputReverse(-.75,10);
		 Lift.Set(ControlMode::PercentOutput,Manipulator.GetY(XboxController::kRightHand));
	 }

}
void IntakeControl(){
	IntakeLeft.Set(ControlMode::PercentOutput,IntakeSpeed);
			IntakeRight.Set(ControlMode::PercentOutput,-1*IntakeSpeed);
			if(Driver.GetBumper(XboxController::kLeftHand)==1 and Driver.GetAButton()!=1 and Driver.GetYButton()!=1){
					IntakeSpeed = -.75;
				}else if (Driver.GetAButton()==1 and Manipulator.GetBumper(XboxController::kLeftHand)!=1 and Driver.GetYButton()!= 1){
					IntakeSpeed = 1;
				}else if (Driver.GetAButton()!=1 and Manipulator.GetBumper(XboxController::kLeftHand)!=1 and Driver.GetYButton()== 1){
					IntakeSpeed = .25;
				}else if(Driver.GetXButton()) {IntakeSpeed = .5;}
				else{
					IntakeSpeed = -.1;
				}
	 }
void LSAMESIDESWITCHPLUSAPICKUP(){switch(testing){
		case(0):
		testing = 1;
		drivetrain1.reset();
		Gyro.Reset();
		break;
		case(1):

				if(LeftDrive.GetSelectedSensorPosition(0)>-10000 and LeftDrive.GetSelectedSensorPosition(0)<-2000){
					SpeedB.Set(DoubleSolenoid::kForward);
				}else{
					SpeedB.Set(DoubleSolenoid::kReverse);
				}
			drivetrain1.DriveStraight(140,.5);
		if(drivetrain1.InPosition(2)==1 and LeftDrive.GetSelectedSensorVelocity(0)<=0 ){
			testing = 2;
			drivetrain1.reset();
			Gyro.Reset();
			loopcount = 0;
		}
		break;

			case(2):

							drivetrain1.DriveTurn(91,.5);
							if(drivetrain1.InPosition(2)==1  and LeftDrive.GetSelectedSensorVelocity(0)<=0){
								testing 	= 3;
								drivetrain1.reset();
								loopcount = 0;
			}

			break;
		case(3):
		Lift.Set(ControlMode::Position,4000);
					if(Lift.GetSelectedSensorPosition(0)>=3000){
						IntakeSpeed = .75;
					}else{
						IntakeSpeed = -.1;
					}
			drivetrain1.DriveStraight(20,.5);
		if(drivetrain1.InPosition(2)==1 or loopcount >=20){
			testing = 4;
			drivetrain1.reset();
			Gyro.Reset();

		}
		break;
		case(4):

				drivetrain1.DriveStraight(-20,.5);

		IntakeSpeed = -.1;
		if(drivetrain1.InPosition(3)==1 /*and LeftDrive.GetSelectedSensorVelocity(0)<=5*/){
			testing = 5;
			drivetrain1.reset();
		}
		break;
		case(5):

		Lift.Set(ControlMode::Position,000);
		drivetrain1.DriveTurn(-89,.5);
		if(drivetrain1.InPosition(3)==1){
			testing = 6;
			drivetrain1.reset();
			Seconds = 0;
			loopcount = 0;
			loopcount = 0;
			loopcount = 0;
			loopcount = 0;
		}
		break;
			case(6):

		drivetrain1.DriveStraight(90,.5);
		Lift.Set(ControlMode::Position,0 );

		if(drivetrain1.InPosition(2)==1){
							testing = 7;
							drivetrain1.reset();
							Gyro.Reset();
						}
		break;
			case(7):
		SpeedB.Set(DoubleSolenoid::kForward);
					IntakeSpeed = 0;
					Lift.Set(ControlMode::Position,0);
					drivetrain1.DriveTurn(140,.5);//130
					if(drivetrain1.InPosition(3)==1){
						testing = 8;
						drivetrain1.reset();
						loopcount = 0;
					}
					break;
			case(8):
		LeftDrive.ConfigPeakOutputForward(.5,10);
			 					 				RightDrive.ConfigPeakOutputForward(.85,10);
			 					 				LeftDrive.ConfigPeakOutputReverse(-.5,10);
			 					 						RightDrive.ConfigPeakOutputReverse(-.85,10);
					IntakeSpeed = -.75;
							Lift.Set(ControlMode::Position,0);
							drivetrain1.DriveStraight(54,.5);
							SpeedB.Set(DoubleSolenoid::kForward);
							if(drivetrain1.InPosition(3)==1 or loopcount >=50){
								testing = 9;
								drivetrain1.reset();
								loopcount = 0;
							}
							break;
			case(9):
		drivetrain1.DriveStraight(-5,.5);
							SpeedB.Set(DoubleSolenoid::kForward);
							IntakeSpeed = -.5;
							if(loopcount >= 20){
								testing = 10;
								SpeedB.Set(DoubleSolenoid::kReverse);
								drivetrain1.reset();
							}
		break;
			case(10):
								IntakeSpeed = -.15;
										Lift.Set(ControlMode::Position,0);
										drivetrain1.DriveTurn(-40,.5);
										SpeedB.Set(DoubleSolenoid::kReverse);
										if(drivetrain1.InPosition(2)==1){
											testing = 11;
											drivetrain1.reset();
										}
										break;
			case(11):
								IntakeSpeed = -.1;
										Lift.Set(ControlMode::Position,0);
										drivetrain1.DriveStraight(200,.5);
										SpeedB.Set(DoubleSolenoid::kReverse);
										if(drivetrain1.InPosition(2)==1){
											testing = 12;
											drivetrain1.reset();
										}
										break;
			case(12):
								IntakeSpeed = 0;
			LeftDrive.Set(ControlMode::PercentOutput,0);
										RightDrive.Set(ControlMode::PercentOutput,0);
									//	Lift.Set(ControlMode::Position,0);
									//	drivetrain1.DriveStraight(36);
									//	SpeedB.Set(DoubleSolenoid::kForward);
										/*	if(drivetrain1.InPosition(2)==1){
											testing = 9;
											drivetrain1.reset();
										}*/
										break;

		}}
void LFARSIDESCALE(){
		switch(testing){
		case(0):
		testing = 1;
		drivetrain1.reset();
		Gyro.Reset();
		break;
		case(1):

				if(LeftDrive.GetSelectedSensorPosition(0)>-10000 and LeftDrive.GetSelectedSensorPosition(0)<-2000){
					SpeedB.Set(DoubleSolenoid::kForward);
				}else{
					SpeedB.Set(DoubleSolenoid::kReverse);
				}
			drivetrain1.DriveStraight(206,.5);//200
			IntakeSpeed = -.1;
		if(drivetrain1.InPosition(2)==1 and LeftDrive.GetSelectedSensorVelocity(0)<=0 ){
			testing = 2;
			drivetrain1.reset();
			Gyro.Reset();
			loopcount = 0;
		}
		break;

			case(2):

							drivetrain1.DriveTurn(83,.5);//91//90
							if((drivetrain1.InPosition(2)==1 /*3*/ and LeftDrive.GetSelectedSensorVelocity(0)<=0 )){
								testing 	= 3;
								drivetrain1.reset();

			}

			break;
		case(3):

			drivetrain1.DriveStraight(205,.5);
		if((drivetrain1.InPosition(2)==1 and LeftDrive.GetSelectedSensorVelocity(0)<=0) ){
			testing = 4;
			drivetrain1.reset();
			Gyro.Reset();
			loopcount = 0;
		}
		break;
		case(4):

				drivetrain1.DriveTurn(-110,.5);
		Lift.Set(ControlMode::Position,00);
		IntakeSpeed = -.1;
		if(drivetrain1.InPosition(4)==1 ){
			testing = 5;
			drivetrain1.reset();
			loopcount = 0;
		}
		break;
		case(5):

		Lift.Set(ControlMode::Position,0);
		drivetrain1.DriveStraight(23,.5);//25
		if(drivetrain1.InPosition(2)==1){
			testing = 6;
			drivetrain1.reset();
			Seconds = 0;
			loopcount = 0;
			loopcount = 0;
			loopcount = 0;
			loopcount = 0;
		}
		break;
			case(6):
		Lift.Set(ControlMode::Position,19000 );

		drivetrain1.STOP();

		if(Lift.GetSelectedSensorPosition(0)>=16000){
			IntakeSpeed = .6;
		}else{
			IntakeSpeed = -.1;
			loopcount = 0;
		}
		if(Lift.GetSelectedSensorPosition(0)>=16000 and IntakeSpeed ==.75 and loopcount >=20){
							testing = 7;
							drivetrain1.reset();
							Gyro.Reset();
						}
		break;
			case(7):

					IntakeSpeed = 0;
					Lift.Set(ControlMode::Position,0);
					Lift.Set(ControlMode::Position,0);
					Lift.Set(ControlMode::Position,0);
					drivetrain1.DriveStraight(-30,.5);//-120 turn
					if(drivetrain1.InPosition(2)==1){
						testing = 8;
						drivetrain1.reset();
					}
					break;
			case(8):

					IntakeSpeed = -.5;
							Lift.Set(ControlMode::Position,0);
							drivetrain1.DriveTurn(-90,.5);//-120
							SpeedB.Set(DoubleSolenoid::kForward);
							if(drivetrain1.InPosition(2)==1){
								testing = 12;
								drivetrain1.reset();
							}
							break;
			case(9):
		LeftDrive.Set(ControlMode::PercentOutput,0);
							RightDrive.Set(ControlMode::PercentOutput,0);
							SpeedB.Set(DoubleSolenoid::kReverse);
							IntakeSpeed = -.1;
		break;
			case(10):
								IntakeSpeed = -.5;
										Lift.Set(ControlMode::Position,0);
										drivetrain1.DriveStraight(36,.5);
										SpeedB.Set(DoubleSolenoid::kForward);
										if(drivetrain1.InPosition(2)==1){
											testing = 9;
											drivetrain1.reset();
										}
										break;}
	}
void SmartDash(){
		SmartDashboard::PutNumber("gyro", Gyro.GetAngle());
				SmartDashboard::PutNumber("Lift Encoder", Lift.GetSelectedSensorPosition(0));
				SmartDashboard::PutNumber("Left Drive Encoder", LeftDrive.GetSelectedSensorPosition(0));
				SmartDashboard::PutNumber("Right Drive Encoder", RightDrive.GetSelectedSensorPosition(0));

				SmartDashboard::PutNumber("Lift Voltage", Lift.GetMotorOutputVoltage());
				SmartDashboard::PutNumber("Lift Current",Lift.GetOutputCurrent());
				SmartDashboard::PutNumber("Lift1 Voltage", Lift1.GetMotorOutputVoltage());
				SmartDashboard::PutNumber("Lift1 Current", Lift1.GetOutputCurrent());

				SmartDashboard::PutNumber("VelocityCorrection",VelocityCorrection);
				SmartDashboard::PutNumber("Intake Status", IntakeStatus);
				SmartDashboard::PutNumber("Speed Bar Position",SpeedB.Get());
				SmartDashboard::PutNumber("Overide",IntakeOveride);
				SmartDashboard::PutNumber("Intakeroo",Intakeroo);
				SmartDashboard::PutNumber("AutoStatus", AutoStatus);
				SmartDashboard::PutNumber("velocity",LeftDrive.GetSelectedSensorVelocity(0));
				SmartDashboard::PutNumber("position",RightDrive.GetSelectedSensorPosition(0));
				SmartDashboard::PutNumber("PositionR", RightDrive.GetSelectedSensorPosition(0)/4096);
				SmartDashboard::PutNumber("RightLead",RightDrive.GetOutputCurrent());
				SmartDashboard::PutNumber("RightFollow1",RightDrive1.GetOutputCurrent());
				SmartDashboard::PutNumber("RightFollow2",RightDrive2.GetOutputCurrent());

	}
void TeleopInit() {
		LeftDrive.SetSelectedSensorPosition(0,0,10);
		RightDrive.SetSelectedSensorPosition(0,0,10);
		Record.open("/home/lvuser/Recordings/Recordings.csv", std::ofstream::out | std::ofstream::app);
		LeftDrive.ClearMotionProfileHasUnderrun(10);
		RightDrive.ClearMotionProfileHasUnderrun(10);
		Gyro.Reset();

	}
void LTWOCUBESCALESAMESIDE(){
	switch(testing){
		case(0):
		testing = 1;
		drivetrain1.reset();
		Gyro.Reset();
		loopcount = 0;
		break;
		case(1):

				if(LeftDrive.GetSelectedSensorPosition(0)>-12000 and LeftDrive.GetSelectedSensorPosition(0)<-1000){
					SpeedB.Set(DoubleSolenoid::kForward);
				}else{
					SpeedB.Set(DoubleSolenoid::kReverse);
				}
			drivetrain1.DriveStraight(238,.75);
			IntakeSpeed = -.1;
		if((drivetrain1.InPosition(4)==1 and LeftDrive.GetSelectedSensorVelocity(0)==0) or loopcount >=80 ){
			testing = 2;
			drivetrain1.reset();
			drivetrain1.reset();
			Gyro.Reset();
			loopcount = 0;
			loopcount = 0;
			loopcount = 0;
		}

		break;

			case(2):
		IntakeSpeed = -.1;
							drivetrain1.DriveTurn(38,.5);//44
							if((drivetrain1.InPosition(2)==1 and LeftDrive.GetSelectedSensorVelocity(0)<=0)or loopcount >= 50){
								testing 	= 3;
								drivetrain1.reset();
								drivetrain1.STOP();
								loopcount = 0;
			}

			break;
		case(3):
		Lift.Set(ControlMode::Position,19000);
					if(Lift.GetSelectedSensorPosition(0)>=18000 ){
						IntakeSpeed = .75;
					}else{
						IntakeSpeed = -.1;

					}
					drivetrain1.STOP();
		if( loopcount >=50 and Lift.GetSelectedSensorPosition(0)>=18000){
			testing = 4;
			drivetrain1.reset();
			Gyro.Reset();
			loopcount = 0;

		}
		break;
		//turn for second cube
		case(4):
		Lift.Set(ControlMode::Position,0);
				drivetrain1.DriveTurn(90,.5);//106

		IntakeSpeed = .75;
		if(drivetrain1.InPosition(2)==1 or loopcount >= 75){
			testing = 5;
			drivetrain1.reset();
			loopcount = 0;
			loopcount = 0;
		}
		break;
		case(5):

		Lift.Set(ControlMode::Position,000);
		drivetrain1.DriveStraight(62,.35);
		SpeedB.Set(DoubleSolenoid::kForward);
		IntakeSpeed = -.75;
		if(drivetrain1.InPosition(3)==1 or loopcount >= 55){
			testing = 6;
			drivetrain1.reset();
			Seconds = 0;
			loopcount = 0;
			loopcount = 0;
			loopcount = 0;
			loopcount = 0;
		}
		break;
			case(6):

		drivetrain1.DriveStraight(-50,.5);
		Lift.Set(ControlMode::Position,0 );
		IntakeSpeed=-.5;
		SpeedB.Set(DoubleSolenoid::kReverse);
		if(drivetrain1.InPosition(2)==1 or loopcount>=50){
							testing = 7;
							drivetrain1.reset();
							Gyro.Reset();
						}
		break;
			case(7):
		SpeedB.Set(DoubleSolenoid::kReverse);
					IntakeSpeed = -.25;
					Lift.Set(ControlMode::Position,0);
					drivetrain1.DriveTurn(-88,.5);//95
					if(drivetrain1.InPosition(3)==1 or loopcount >= 70){
						testing = 8;
						drivetrain1.reset();
						loopcount = 0;
						loopcount = 0;
									loopcount = 0;
									loopcount = 0;
					}
					break;
			case(8):

												Lift.Set(ControlMode::Position,19000);
							if(Lift.GetSelectedSensorPosition(0)>=17000){
								IntakeSpeed = .75;//.75
							}else{
								IntakeSpeed = -.1;
							}
							drivetrain1.STOP();
							SpeedB.Set(DoubleSolenoid::kReverse);
							if( loopcount >=200 and Lift.GetSelectedSensorPosition(0)>=18500){
								testing = 12;
								drivetrain1.reset();
								loopcount = 0;
							}
							break;
			case(9):
		LeftDrive.Set(ControlMode::PercentOutput,0);
							RightDrive.Set(ControlMode::PercentOutput,0);
							SpeedB.Set(DoubleSolenoid::kForward);
							IntakeSpeed = -.5;
							if(loopcount >= 20){
								testing = 10;
								SpeedB.Set(DoubleSolenoid::kReverse);
								drivetrain1.reset();
							}
		break;
			case(10):
								IntakeSpeed = -.15;
										Lift.Set(ControlMode::Position,0);
										drivetrain1.DriveTurn(-40,.5);
										SpeedB.Set(DoubleSolenoid::kReverse);
										if(drivetrain1.InPosition(2)==1){
											testing = 11;
											drivetrain1.reset();
										}
										break;
			case(11):
								IntakeSpeed = -.1;
										Lift.Set(ControlMode::Position,0);
										drivetrain1.DriveStraight(200,.5);
										SpeedB.Set(DoubleSolenoid::kReverse);
										if(drivetrain1.InPosition(2)==1){
											testing = 12;
											drivetrain1.reset();
										}
										break;
			case(12):
								IntakeSpeed = 0;
			drivetrain1.STOP();

										break;

		}

}
void LTWOCUBESWITCHANDSCALESAMESIDE(){switch(testing){
		case(0):
		testing = 1;
		drivetrain1.reset();
		Gyro.Reset();
		loopcount = 0;
		break;
		case(1):

				if(LeftDrive.GetSelectedSensorPosition(0)>-12000 and LeftDrive.GetSelectedSensorPosition(0)<-1000){
					SpeedB.Set(DoubleSolenoid::kForward);
				}else{
					SpeedB.Set(DoubleSolenoid::kReverse);
				}
			drivetrain1.DriveStraight(233,.75);
		if((drivetrain1.InPosition(4)==1 and LeftDrive.GetSelectedSensorVelocity(0)==0) or loopcount >=80 ){
			testing = 2;
			drivetrain1.reset();
			Gyro.Reset();
			loopcount = 0;
		}

		break;

			case(2):

							drivetrain1.DriveTurn(40,.5);
							if((drivetrain1.InPosition(3)==1 and LeftDrive.GetSelectedSensorVelocity(0)<=0)or loopcount >= 50){
								testing 	= 3;
								drivetrain1.reset();
								loopcount = 0;
			}

			break;
		case(3):
		Lift.Set(ControlMode::Position,19000);
					if(Lift.GetSelectedSensorPosition(0)>=18200){
						IntakeSpeed = 1;
					}else{
						IntakeSpeed = -.1;
					}
					LeftDrive.Set(ControlMode::PercentOutput,0);
					RightDrive.Set(ControlMode::PercentOutput,0);
		if( loopcount >=50 and Lift.GetSelectedSensorPosition(0)>=18400){
			testing = 4;
			drivetrain1.reset();
			Gyro.Reset();
			loopcount = 0;

		}
		break;
		case(4):
		Lift.Set(ControlMode::Position,0);
				drivetrain1.DriveTurn(107,.5);

		IntakeSpeed = .75;
		if(drivetrain1.InPosition(2)==1 or loopcount >= 40){
			testing = 5;
			drivetrain1.reset();
			loopcount = 0;
			loopcount = 0;
		}
		break;
		case(5):

		Lift.Set(ControlMode::Position,000);
		drivetrain1.DriveStraight(57,.35);
		SpeedB.Set(DoubleSolenoid::kForward);
		IntakeSpeed = -.75;
		if(drivetrain1.InPosition(3)==1 or loopcount >= 55){
			testing = 6;
			drivetrain1.reset();
			Seconds = 0;
			loopcount = 0;
			loopcount = 0;
			loopcount = 0;
			loopcount = 0;
		}
		break;
			case(6):

		drivetrain1.DriveStraight(-6,.5);
		Lift.Set(ControlMode::Position,0 );
		IntakeSpeed=-.2;
		SpeedB.Set(DoubleSolenoid::kReverse);
		if(drivetrain1.InPosition(2)==1 or loopcount>=20){
							testing = 7;
							drivetrain1.reset();
							Gyro.Reset();
						}
		break;
			case(7):
		SpeedB.Set(DoubleSolenoid::kReverse);
					IntakeSpeed = -.10;
					Lift.Set(ControlMode::Position,4000);
					drivetrain1.DriveStraight(18,.5);
					if(drivetrain1.InPosition(2)==1 or loopcount >= 70){
						testing = 8;
						drivetrain1.reset();
						loopcount = 0;
					}
					break;
			case(8):

												Lift.Set(ControlMode::Position,4000);
							if(Lift.GetSelectedSensorPosition(0)>=3000){
								IntakeSpeed = 1;
							}else{
								IntakeSpeed = -.1;
							}
							LeftDrive.Set(ControlMode::PercentOutput,0);
												RightDrive.Set(ControlMode::PercentOutput,0);
							SpeedB.Set(DoubleSolenoid::kReverse);
							if( loopcount >=50){
								testing = 12;
								drivetrain1.reset();
								loopcount = 0;
							}
							break;
			case(9):
		LeftDrive.Set(ControlMode::PercentOutput,0);
							RightDrive.Set(ControlMode::PercentOutput,0);
							SpeedB.Set(DoubleSolenoid::kForward);
							IntakeSpeed = -.5;
							if(loopcount >= 20){
								testing = 10;
								SpeedB.Set(DoubleSolenoid::kReverse);
								drivetrain1.reset();
							}
		break;
			case(10):
								IntakeSpeed = -.15;
										Lift.Set(ControlMode::Position,0);
										drivetrain1.DriveTurn(-40,.5);
										SpeedB.Set(DoubleSolenoid::kReverse);
										if(drivetrain1.InPosition(2)==1){
											testing = 11;
											drivetrain1.reset();
										}
										break;
			case(11):
								IntakeSpeed = -.1;
										Lift.Set(ControlMode::Position,0);
										drivetrain1.DriveStraight(200,.5);
										SpeedB.Set(DoubleSolenoid::kReverse);
										if(drivetrain1.InPosition(2)==1){
											testing = 12;
											drivetrain1.reset();
										}
										break;
			case(12):
								IntakeSpeed = 0;
			LeftDrive.Set(ControlMode::PercentOutput,0);
										RightDrive.Set(ControlMode::PercentOutput,0);
									//	Lift.Set(ControlMode::Position,0);
									//	drivetrain1.DriveStraight(36);
									//	SpeedB.Set(DoubleSolenoid::kForward);
										/*	if(drivetrain1.InPosition(2)==1){
											testing = 9;
											drivetrain1.reset();
										}*/
										break;

		}
		SmartDashboard::PutNumber("Testing",testing);}
void TeleopPeriodic() {
	if(Driver.GetBumperPressed(XboxController::kRightHand)==1){
				Reverse = !Reverse;

			}
	SmartDashboard::PutBoolean("Direction",Reverse);
	if(Driver.GetBumperPressed(XboxController::kLeftHand)==1){
					drivetrain1.HOLD();

				}else{drivetrain1.TeleopDrive(.5,Reverse,Driver.GetY(XboxController::kLeftHand),Driver.GetY(XboxController::kRightHand));
}


		IntakeControl();
		LiftControl();
		SmartDash();




		SmartDashboard::PutNumber("Testing",testing);
		SmartDashboard::PutNumber("Intakeroo",Intakeroo);
		bool There;

				if(drivetrain1.InPosition(2)==1){
					There = 1;
				}else{
					There = 0;
				}


				SmartDashboard::PutNumber("There",There);
				SmartDashboard::PutNumber("LOOPS",loopcount);

		Climb2.Set(ControlMode::PercentOutput,Climb1.GetMotorOutputPercent());
		Climb3.Set(ControlMode::PercentOutput,Climb1.GetMotorOutputPercent());
		Climb4.Set(ControlMode::PercentOutput,Climb1.GetMotorOutputPercent());

	/*	ProfileL.PeriodicTask();
		ProfileR.PeriodicTask();
		ProfileL.control();
		ProfileR.control();*/
		double LeftRotations =LeftDrive.GetSelectedSensorPosition(0);
		double velocityL = LeftDrive.GetSelectedSensorVelocity(0);
		double velocity = RightDrive.GetSelectedSensorVelocity(0);
		double RightRotations = RightDrive.GetSelectedSensorPosition(0);
		VelL = (velocityL)/4096*600;
		VelR = (velocity)/4096*600;
		PositionR = (RightRotations/4096)*1.0;
		PositionL = (LeftRotations/4096)*1.0;
		VelocityCorrection = (velocityL-velocity)/5000;
		double PositionOffset = (LeftRotations-RightRotations)/1000;
		std::shared_ptr<NetworkTable> table =   NetworkTable::GetTable("limelight");
		/*float targetOffsetAngle_Horizontal = table->GetNumber("tx",0);
		float targetOffsetAngle_Vertical = table->GetNumber("ty",0);
		float targetArea = table->GetNumber("ta",0);
		float targetSkew = table->GetNumber("ts",0);*/
		float camera = table->GetNumber("camMode",0);
		//float gotTarget = table->GetNumber("tv",0);

							SmartDashboard::PutNumber("Offset",PositionOffset);
	//	SmartDashboard::PutNumber("LimeLight",targetOffsetAngle_Vertical);
		SmartDashboard::PutNumber("Cam Mode",camera);


		loopcount = loopcount+1;
		double Seconds = loopcount/50;














		GyroCorrection = Gyro.GetAngle()/1000;










	if(Lift.GetSelectedSensorPosition(0)>=3250){
		SpeedB.Set(DoubleSolenoid::kReverse);
		Intakeroo = 0;
	}else{

		if(Driver.GetBButtonPressed()==1){
					Intakeroo = !Intakeroo;
				}
				if(Intakeroo==1){
					SpeedB.Set(DoubleSolenoid::kForward);
				}else{
					SpeedB.Set(DoubleSolenoid::kReverse);
				}

	}
	if(Manipulator.GetBumper(XboxController::kRightHand)==1){
		ClimbRelease.Set(true);}
	else{ClimbRelease.Set(false);}

	if(Manipulator.GetStartButton()==1)
				{Climb.Set(true);}
				else{
					Climb.Set(false);
				}
					if(Manipulator.GetBackButton()==1){
						Climb1.Set(ControlMode::PercentOutput,-1);
					}	else{
						Climb1.Set(ControlMode::PercentOutput,0);
					}
	}



void TestPeriodic() {




	}

private:


	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kRight = "Right";
	const std::string kLeft = "Left";
	const std::string kCenter = "Center";
	const std::string kStraight = "Straight";
	const std::string kLScale = "LScale";
	const std::string kRScale = "RScale";
	const std::string kOURSIDEL = "kOURSIDEL";

	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
