/*
 * Robot.cpp
 *
 *  Created on: Feb 11, 2017
 *      Author: Sameer Vijay
 */

#include <iostream>
#include <memory>
#include <string>
#include <sstream>

#include <Joystick.h>
#include <SampleRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <RobotDrive.h>
#include <Timer.h>
#include <CANTalon.h>
#include <DigitalInput.h>
#include <DriverStation.h>
#include <AHRS.h>
#include <SFDrive.h>
#include <VisionHelper.h>
#include <thread>

#define tickRateForward 325
// 100 is okay for large turns; smaller turns need smaller tick rates
// 35 is good for small angles
#define tickRateTurnBig 100
#define tickRateTurnSmall 35
#define wheelDiameter 4

#define driveTurnConstant 0.0

/*
 * Actual robot ID's
#define versaLID 1
#define frontLID 2
#define backLID 3
#define frontRID 6
#define backRID 5
#define versaRID 4
#define climberID 7

Talon ID Configuration on Robot
	5	1
	6	2
	7	3
	8	4
 */

#define versaLID 7
#define frontLID 5
#define backLID 6
#define frontRID 1
#define backRID 2
#define versaRID 8
#define climberID 3
#define lightID 4

class Robot: public frc::SampleRobot {

	enum AutoState {
		STATE0,
		STATE1,
		STATE2,
		STATE3,
		STATE4,
		STATE5,
		STATE6,
		STATE7,
		STATE8,
		STATE9,
		STOP,
	};
	AutoState autoState;

	SFDrive *sfDrive;

	CANTalon *frontMotorL;
	CANTalon *frontMotorR;
	CANTalon *backMotorL;
	CANTalon *backMotorR;
	CANTalon *versaMotorL;
	CANTalon *versaMotorR;
	CANTalon *climbingMotor;
	CANTalon *lightTalon;

	DoubleSolenoid *rampSolenoid;
	DoubleSolenoid *dropOffSolenoid;
	DoubleSolenoid *climberSolenoid;

	bool autoInit, teleopInit;
	double initialTime, autoStartingAngle, angleToTurn;
	double lastDistanceExecuted;
	double timeDrop, debounceTime;

	Joystick *joystickMain;
	Joystick *joystickSecond;

	VisionHelper *visionHelper;

	frc::DigitalInput *gearLimitSwitch;

	AHRS *navX;

	int lastDistance, lastAngle, lastReceived;
	double lastReceivedTime;

	int forwardCycles;

	frc::SendableChooser<std::string> chooser;
	const std::string autoNothing = "Nothing";
	const std::string autoForward = "DriveForward";
	const std::string autoGearF = "GearForward";
	const std::string autoGearL = "GearLeft";
	const std::string autoGearR = "GearRight";
	const std::string autoHighGoal = "HighGoal";

public:
	Robot() {
		//Note SmartDashboard is not initialized here, wait until RobotInit to make SmartDashboard calls
		//		drive->SetExpiration(0.1);
	}

	void RobotInit() {

		autoInit = false;
		teleopInit = false;

		//		autoState = STATE1;

		frontMotorL = new CANTalon(frontLID);
		backMotorL = new CANTalon(backLID);
		frontMotorR = new CANTalon(frontRID);
		backMotorR = new CANTalon(backRID);
		versaMotorL = new CANTalon(versaLID);
		versaMotorR = new CANTalon(versaRID);
		climbingMotor = new CANTalon(climberID);
		lightTalon = new CANTalon(lightID);

		rampSolenoid = new DoubleSolenoid(5, 1);
		dropOffSolenoid = new DoubleSolenoid(6, 7);
		climberSolenoid = new DoubleSolenoid(2, 3);

		navX = new AHRS(SerialPort::kMXP);

		sfDrive = new SFDrive(frontMotorL, frontMotorR, backMotorL, backMotorR);
		sfDrive->navX = this->navX;

		joystickMain = new Joystick(0);
		joystickSecond = new Joystick(1);

		visionHelper = new VisionHelper();
		//		DriverStation::ReportError(visionHelper->description);

		// PORT NEEDS TO BE CHANGED
		gearLimitSwitch = new DigitalInput(0);

		lastDistance = 0;
		lastAngle = 0;

		timeDrop = 0;
		debounceTime = 0;

		forwardCycles = 0;

		chooser.AddDefault(autoNothing, autoNothing);
		chooser.AddObject(autoForward, autoForward);
		chooser.AddObject(autoGearF, autoGearF);
		chooser.AddObject(autoGearL, autoGearL);
		chooser.AddObject(autoGearR, autoGearR);
		chooser.AddObject(autoHighGoal, autoHighGoal);
		SmartDashboard::PutData("Auto Modes2", &chooser);

		SmartDashboard::PutNumber("SpeedSet", 10);
	}

	void autonomousInit() {
		//		timeInitial = Timer::GetFPGATimestamp();

		autoState = STATE1;

		frontMotorL->SetEncPosition(0);
		frontMotorR->SetEncPosition(0);
		backMotorL->SetEncPosition(0);
		backMotorR->SetEncPosition(0);

		//				navX->Reset();
		autoStartingAngle = navX->GetAngle();
		DriverStation::ReportError("Starting Angle: " + std::to_string(autoStartingAngle));

		//		initialY = imu->GetAngleY();
		//		initialZ = imu->GetAngleZ() - 12;
		//
		//		timesAuto++;

		initialTime = Timer::GetFPGATimestamp();
		forwardCycles = 0;

		sfDrive->mediumDrive = false;
		sfDrive->setPIDInit(false);

		SmartDashboard::PutData("Auto Modes2", &chooser);

		DriverStation::ReportError("Auto init");
	}
	void Autonomous() {
		while (IsAutonomous()) {
			if (IsEnabled()) {
				auto autoSelected = chooser.GetSelected();
				// std::string autoSelected = frc::SmartDashboard::GetString("Auto Selector", autoNameDefault);
				DriverStation::ReportError(autoSelected);
				if (autoInit == false) {
					autoInit = true;
					autonomousInit();
				}
				SmartDashboard::PutNumber("NavX Angle", navX->GetAngle());
				SmartDashboard::PutNumber("Ticks 1", backMotorL->GetEncPosition());
				//				SmartDashboard::PutNumber("SetpointLeft", ticksBackL);
				SmartDashboard::PutNumber("Setpoint 1", sfDrive->getSetpointLeft());
				SmartDashboard::PutNumber("Total Ticks", sfDrive->totalTicks);
				SmartDashboard::PutNumber("Error 1", abs(sfDrive->getSetpointLeft()) - abs(backMotorL->GetEncPosition()));
				//				SmartDashboard::PutNumber("Current Setpoint Left", sfDrive->getSetpointLeft());
				//				SmartDashboard::PutNumber("Current Setpoint Right", sfDrive->getSetpointRight());
				//				SmartDashboard::PutNumber("Current P", backMotorL->GetP());
				//				SmartDashboard::PutNumber("Current I", backMotorL->GetI());
				//				SmartDashboard::PutNumber("Current D", backMotorL->GetD());
				//				SmartDashboard::PutNumber("Left Back", backMotorL->GetEncPosition());
				//				SmartDashboard::PutNumber("Right Back", backMotorR->GetEncPosition());

				//				std::string ticksAndTime = std::to_string(backMotorL->GetEncPosition()) + " "+ std::to_string(Timer::GetFPGATimestamp());
				//				DriverStation::ReportError(ticksAndTime);

				lightTalon->Set(SmartDashboard::GetNumber("LightBright", 0)/12.0);
				//				lightTalon->Set(7.53/12.0);

				SmartDashboard::PutNumber("LastReceived", lastReceived);
				if (visionHelper->checkPendingPacket(&lastReceived) == true) {
					lastReceivedTime = Timer::GetFPGATimestamp();
				}

				if (autoSelected == autoNothing) {

				} else if (autoSelected == autoForward) {
					if (autoState == STATE1) {
						DriverStation::ReportError("State 1");

						dropOffSolenoid->Set(DoubleSolenoid::kForward);
						rampSolenoid->Set(DoubleSolenoid::kForward);

						autoState = STATE2;
					} else if (autoState == STATE2) {
						sfDrive->mediumDrive = true;
						if (sfDrive->driveDistance(150) == true) {
							autoState = STOP;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
							sfDrive->mediumDrive = false;
						}
					} else if (autoState == STOP) {
						DriverStation::ReportError("Auto Complete");
					}
				} else if (autoSelected == autoGearF) {
					if (autoState == STATE1) {
						// Pull drop-off pneumatic up and ramp down in case it isn't already
						DriverStation::ReportError("State 1");

						dropOffSolenoid->Set(DoubleSolenoid::kForward);
						rampSolenoid->Set(DoubleSolenoid::kForward);

						sfDrive->setPIDInit(false);
						initialTime = Timer::GetFPGATimestamp();
						autoState = STATE2;
					} else if (autoState == STATE2) {
						DriverStation::ReportError("State 2");
						sfDrive->mediumDrive = true;
						// Drive distance - half robot length + 4 in buffer
						if (sfDrive->driveDistance(110 - 34/2 - 3) == true) {
							autoState = STATE3;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
							sfDrive->mediumDrive = false;
						}
						if (Timer::GetFPGATimestamp() > initialTime + 4) {
							autoState = STATE3;
							initialTime = Timer::GetFPGATimestamp();
						}
					} else if (autoState == STATE3) {
						// Push drop off pneumatic down and ramp up; hopefully gear is on peg here; wait for a second
						DriverStation::ReportError("State 3");

						if (dropOffSolenoid->Get() == DoubleSolenoid::kForward)
							dropOffSolenoid->Set(DoubleSolenoid::kReverse);
						if (rampSolenoid->Get() == DoubleSolenoid::kForward)
							rampSolenoid->Set(DoubleSolenoid::kReverse);

						if (Timer::GetFPGATimestamp() > initialTime + 1) {
							autoState = STATE4;
							sfDrive->setArcadeInit(false);
							initialTime = Timer::GetFPGATimestamp();
						}
					} else if (autoState == STATE4) {
						DriverStation::ReportError("State 4");
						//						sfDrive->mediumDrive = true;
						//						if (sfDrive->driveDistance(-42) == true) {
						//							autoState = STATE5;
						//							sfDrive->setPIDInit(false);
						//							initialTime = Timer::GetFPGATimestamp();
						//						}
						if (Timer::GetFPGATimestamp() < initialTime + 1.5) {
							sfDrive->joystickDrive(0.5, 0.0);
						} else {
							autoState = STATE5;
						}
					} else if (autoState == STATE5) {
						// Pull ramp down and drop offs up to get ready for a gear cycle in teleop
						DriverStation::ReportError("State 5");

						if (dropOffSolenoid->Get() == DoubleSolenoid::kReverse)
							dropOffSolenoid->Set(DoubleSolenoid::kForward);
						if (rampSolenoid->Get() == DoubleSolenoid::kReverse)
							rampSolenoid->Set(DoubleSolenoid::kForward);

						autoState = STOP;
					} else if (autoState == STOP) {
						DriverStation::ReportError("Auto Complete");
					}
				} else if (autoSelected == autoGearL) {
					if (autoState == STATE1) {
						// Pull drop-off pneumatic up and ramp down in case it isn't already
						DriverStation::ReportError("State 1");

						dropOffSolenoid->Set(DoubleSolenoid::kForward);
						rampSolenoid->Set(DoubleSolenoid::kForward);

						sfDrive->setPIDInit(false);
						initialTime = Timer::GetFPGATimestamp();
						autoState = STATE2;
					} else if (autoState == STATE2) {
						// Initial drive forward
						DriverStation::ReportError("State 2");
						if (sfDrive->driveDistance(106 - 34/2 + 2) == true) {
							autoState = STATE3;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
						}
					} else if (autoState == STATE3) {
						// Initial turn to semi-face peg
						DriverStation::ReportError("State 3");
						if (sfDrive->turnToAngle(60) == true) {
							autoState = STATE4;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
						}
					} else if (autoState == STATE4) {
						// Small drive forward after turning to face peg
						DriverStation::ReportError("State 4");
						if (sfDrive->driveDistance(39) == true) {
							autoState = STATE7;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
						}
						if (Timer::GetFPGATimestamp() > initialTime + 2) {
							autoState = STATE7;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
						}
						//					} else if (autoState == STATE4) {
						//						// Waiting to make sure vision returns an angle
						//						DriverStation::ReportError("State 4");
						//						if (Timer::GetFPGATimestamp() > initialTime + 1.5) {
						//							if (Timer::GetFPGATimestamp() - lastReceivedTime < 2) {
						//								autoState = STATE5;
						//								lastAngle = lastReceived;
						//							} else {
						//								autoState = STOP;
						//							}
						//							sfDrive->setPIDInit(false);
						//							initialTime = Timer::GetFPGATimestamp();
						//						}
						//					} else if (autoState == STATE5) {
						//						// Correcting to angle from vision
						//						DriverStation::ReportError("State 5");
						//						if (sfDrive->turnToAngle(lastAngle) == true) {
						//							autoState = STATE6;
						//							sfDrive->setPIDInit(false);
						//							initialTime = Timer::GetFPGATimestamp();
						//						}
						//					} else if (autoState == STATE6) {
						//						// Drive up to peg
						//						DriverStation::ReportError("State 6");
						//						if (sfDrive->driveDistance(25) == true) {
						//							autoState = STATE7;
						//							sfDrive->setPIDInit(false);
						//							initialTime = Timer::GetFPGATimestamp();
						//						}
					} else if (autoState == STATE7) {
						// Push drop off pneumatic down and ramp up; hopefully gear is on peg here; wait for a second
						DriverStation::ReportError("State 7");

						dropOffSolenoid->Set(DoubleSolenoid::kReverse);
						rampSolenoid->Set(DoubleSolenoid::kReverse);

						if (Timer::GetFPGATimestamp() > initialTime + 0.75) {
							autoState = STATE8;
							initialTime = Timer::GetFPGATimestamp();
						}
					} else if (autoState == STATE8) {
						DriverStation::ReportError("State 8");
						if (Timer::GetFPGATimestamp() < initialTime + 1.5) {
							sfDrive->joystickDrive(0.5, 0.0);
						} else {
							autoState = STATE9;
						}
						/*
											if (sfDrive->driveDistance(-42) == true) {
												autoState = STATE9;
												sfDrive->setPIDInit(false);
												initialTime = Timer::GetFPGATimestamp();
											}
						 */
					} else if (autoState == STATE9) {
						// Pull ramp down and drop offs up to get ready for a gear cycle in teleop
						DriverStation::ReportError("State 9");

						if (dropOffSolenoid->Get() == DoubleSolenoid::kReverse)
							dropOffSolenoid->Set(DoubleSolenoid::kForward);
						if (rampSolenoid->Get() == DoubleSolenoid::kReverse)
							rampSolenoid->Set(DoubleSolenoid::kForward);

						autoState = STOP;
					} else if (autoState == STOP) {
						DriverStation::ReportError("Auto Complete");
					}
				} else if (autoSelected == autoGearR) {

					if (autoState == STATE1) {
						// Pull drop-off pneumatic up and ramp down in case it isn't already
						DriverStation::ReportError("State 1");

						dropOffSolenoid->Set(DoubleSolenoid::kForward);
						rampSolenoid->Set(DoubleSolenoid::kForward);

						sfDrive->setPIDInit(false);
						initialTime = Timer::GetFPGATimestamp();
						autoState = STATE2;
					} else if (autoState == STATE2) {
						// Initial drive forward
						DriverStation::ReportError("State 2");
						if (sfDrive->driveDistance(106 - 34/2 + 2) == true) {
							autoState = STATE3;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
						}
					} else if (autoState == STATE3) {
						// Initial turn to semi-face peg
						DriverStation::ReportError("State 3");
						if (sfDrive->turnToAngle(-58) == true) {
							autoState = STATE4;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
						}
					} else if (autoState == STATE4) {
						// Small drive forward after turning to face peg
						DriverStation::ReportError("State 4");
						if (sfDrive->driveDistance(39) == true) {
							autoState = STATE7;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
						}
						if (Timer::GetFPGATimestamp() > initialTime + 2) {
							autoState = STATE7;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
						}
						//					} else if (autoState == STATE4) {
						//						// Waiting to make sure vision returns an angle
						//						DriverStation::ReportError("State 4");
						//						if (Timer::GetFPGATimestamp() > initialTime + 1.5) {
						//							if (Timer::GetFPGATimestamp() - lastReceivedTime < 2) {
						//								autoState = STATE5;
						//								lastAngle = lastReceived;
						//							} else {
						//								autoState = STOP;
						//							}
						//							sfDrive->setPIDInit(false);
						//							initialTime = Timer::GetFPGATimestamp();
						//						}
						//					} else if (autoState == STATE5) {
						//						// Correcting to angle from vision
						//						DriverStation::ReportError("State 5");
						//						if (sfDrive->turnToAngle(lastAngle) == true) {
						//							autoState = STATE6;
						//							sfDrive->setPIDInit(false);
						//							initialTime = Timer::GetFPGATimestamp();
						//						}
						//					} else if (autoState == STATE6) {
						//						// Drive up to peg
						//						DriverStation::ReportError("State 6");
						//						if (sfDrive->driveDistance(25) == true) {
						//							autoState = STATE7;
						//							sfDrive->setPIDInit(false);
						//							initialTime = Timer::GetFPGATimestamp();
						//						}
					} else if (autoState == STATE7) {
						// Push drop off pneumatic down and ramp up; hopefully gear is on peg here; wait for a second
						DriverStation::ReportError("State 7");

						dropOffSolenoid->Set(DoubleSolenoid::kReverse);
						rampSolenoid->Set(DoubleSolenoid::kReverse);

						if (Timer::GetFPGATimestamp() > initialTime + 0.75) {
							autoState = STATE8;
							initialTime = Timer::GetFPGATimestamp();
							sfDrive->setArcadeInit(false);
						}
					} else if (autoState == STATE8) {
						DriverStation::ReportError("State 8");
						if (Timer::GetFPGATimestamp() < initialTime + 1.1) {
							sfDrive->joystickDrive(0.7, 0.0);
						} else {
							autoState = STATE9;
						}
						/*
																if (sfDrive->driveDistance(-42) == true) {
																	autoState = STATE9;
																	sfDrive->setPIDInit(false);
																	initialTime = Timer::GetFPGATimestamp();
																}
						 */
					} else if (autoState == STATE9) {
						// Pull ramp down and drop offs up to get ready for a gear cycle in teleop
						DriverStation::ReportError("State 9");

						if (dropOffSolenoid->Get() == DoubleSolenoid::kReverse)
							dropOffSolenoid->Set(DoubleSolenoid::kForward);
						if (rampSolenoid->Get() == DoubleSolenoid::kReverse)
							rampSolenoid->Set(DoubleSolenoid::kForward);

						autoState = STOP;
					} else if (autoState == STOP) {
						DriverStation::ReportError("Auto Complete");
					}
				} else if (autoSelected == autoHighGoal) {
				}
			} else {
				autoInit = false;
			}
		}
	}

	void teleoperatedInit() {

		DriverStation::ReportError("Teleop Init");

		navX->Reset();

		autoState = STATE0;
		timeDrop = 0;
		debounceTime = 0;

		frontMotorL->SetControlMode(CANSpeedController::kPercentVbus);
		frontMotorR->SetControlMode(CANSpeedController::kPercentVbus);
		backMotorL->SetControlMode(CANSpeedController::kPercentVbus);
		backMotorR->SetControlMode(CANSpeedController::kPercentVbus);

		lightTalon->SetControlMode(CANSpeedController::kPercentVbus);

		versaMotorL->SetControlMode(CANSpeedController::kFollower);
		versaMotorR->SetControlMode(CANSpeedController::kFollower);

		//		backMotorL->SetControlMode(CANSpeedController::kSpeed);
		//		backMotorL->SetSensorDirection(true);
		//		backMotorL->SetPID(SmartDashboard::GetNumber("Current P", 0),
		//				SmartDashboard::GetNumber("Current I", 0),
		//				SmartDashboard::GetNumber("Current D", 0));
		//		SmartDashboard::PutNumber("SetRPM", 0);
		//		DriverStation::ReportError("Putting");

		SmartDashboard::PutData("Auto Modes2", &chooser);

		//		SmartDashboard::PutNumber("LightBright", 7.53);

	}
	void OperatorControl() override {
		//		drive->SetSafetyEnabled(true);
		while (IsOperatorControl()) {
			if (IsEnabled()) {
				if (!teleopInit) {
					teleopInit = true;
					teleoperatedInit();
				}
				SmartDashboard::PutNumber("NavX Angle", navX->GetAngle());
				SmartDashboard::PutNumber("Ticks 1", frontMotorL->GetEncPosition());
				SmartDashboard::PutNumber("Setpoint 1", sfDrive->getSetpointLeft());
				SmartDashboard::PutNumber("Total Ticks", sfDrive->totalTicks);
				SmartDashboard::PutNumber("Error 1", abs(sfDrive->getSetpointLeft()) - abs(frontMotorL->GetEncPosition()));

				SmartDashboard::PutData("Auto Modes2", &chooser);

				if (dropOffSolenoid->Get() == DoubleSolenoid::kForward)
					SmartDashboard::PutString("Drop Off", "Up");
				else
					SmartDashboard::PutString("Drop Off", "Down");

				if (rampSolenoid->Get() == DoubleSolenoid::kForward)
					SmartDashboard::PutString("Ramp", "Down");
				else
					SmartDashboard::PutString("Ramp", "Up");

				/*
				if (gearLimitSwitch->Get() == 0) {
					// Switch Closed - possessing gear
				} else if (gearLimitSwitch->Get() == 1) {
					// Switch Open - not possessing gear
				} else {
					DriverStation::ReportError("Error: Limit switch returning something not 0 or 1");
				}

				 */

				//				DriverStation::ReportError("Calling receive UDP");
				//				string udpReceive = visionHelper->udpReceiveString;
				//				DriverStation::ReportError("Received " + udpReceive + " from " + visionHelper->hostIP);
				//				if (visionHelper->receiveHostIP() == true) {
				//					string udpReceive = visionHelper->udpReceiveString;
				//					DriverStation::ReportError("Received " + udpReceive + " from " + visionHelper->hostIP);
				//				} else {
				//					DriverStation::ReportError("Still nothing");
				//				}
				//				DriverStation::ReportError("After calling receive UDP");
				//				if (visionHelper->hostIPReceived == true) {
				//					visionHelper->receiveTCP();
				//					string tcpReceive = visionHelper->tcpReceiveString;
				//					DriverStation::ReportError("TCP Received " + tcpReceive + " from " + visionHelper->hostIP);
				//				}

				//				if (Timer::GetFPGATimestamp() > debounceTime + 0.5 && Timer::GetFPGATimestamp() > timeDrop + 0.5) {
				//					rampSolenoid->Set(DoubleSolenoid::kForward);
				//				}
				//				if (joystickMain->GetRawButton(4) || joystickSecond->GetRawButton(4)) {
				//					if (rampSolenoid->Get() == DoubleSolenoid::kForward) {
				//						debounceTime = Timer::GetFPGATimestamp();
				//						rampSolenoid->Set(DoubleSolenoid::kReverse);
				//					}
				//				}

				if (Timer::GetFPGATimestamp() > timeDrop + 0.6) {
					rampSolenoid->Set(DoubleSolenoid::kForward);
				}
				if (joystickMain->GetRawButton(1) || joystickSecond->GetRawButton(1)) {
					DriverStation::ReportError("Pulling down");
					timeDrop = Timer::GetFPGATimestamp();
					if (rampSolenoid->Get() == DoubleSolenoid::kForward) {
						rampSolenoid->Set(DoubleSolenoid::kReverse);
					}
				}

				//				if (joystickMain->GetRawButton(1) || joystickSecond->GetRawButton(1)) {
				//					if (rampSolenoid->Get() != DoubleSolenoid::kForward)
				//						rampSolenoid->Set(DoubleSolenoid::kForward);
				//				} else if (joystickMain->GetRawButton(4) || joystickSecond->GetRawButton(4)) {
				//					if (rampSolenoid->Get() != DoubleSolenoid::kReverse)
				//						rampSolenoid->Set(DoubleSolenoid::kReverse);
				//				}



				if (joystickMain->GetRawButton(2) || joystickSecond->GetRawButton(2)) {
					if (dropOffSolenoid->Get() != DoubleSolenoid::kForward)
						dropOffSolenoid->Set(DoubleSolenoid::kForward);
				} else if (joystickMain->GetRawButton(3) || joystickSecond->GetRawButton(3)) {
					if (dropOffSolenoid->Get() != DoubleSolenoid::kReverse)
						dropOffSolenoid->Set(DoubleSolenoid::kReverse);
				}
				if (joystickMain->GetRawButton(5) || joystickSecond->GetRawButton(5)) {
					if (climberSolenoid->Get() != DoubleSolenoid::kForward)
						climberSolenoid->Set(DoubleSolenoid::kForward);
				} else if (joystickMain->GetRawButton(6) || joystickSecond->GetRawButton(6)) {
					if (climberSolenoid->Get() != DoubleSolenoid::kReverse)
						climberSolenoid->Set(DoubleSolenoid::kReverse);
				}

				if (joystickMain->GetRawAxis(3) > 0.1 || joystickSecond->GetRawAxis(3)) {
					climbingMotor->Set(-1);
				} else if (joystickMain->GetRawAxis(2) > 0.1 || joystickSecond->GetRawAxis(2)) {
					climbingMotor->Set(1);
				} else {
					climbingMotor->Set(0);
				}

				lightTalon->Set(abs(SmartDashboard::GetNumber("LightBright", 0) / 12.0));

				SmartDashboard::PutNumber("LastReceived", lastReceived);
				if (visionHelper->checkPendingPacket(&lastReceived) == true) {
					lastReceivedTime = Timer::GetFPGATimestamp();
					DriverStation::ReportError("Received " + std::to_string(lastReceived));
				} else {
					DriverStation::ReportError("Received nothing");
				}

				if (joystickMain->GetRawButton(4)) {
					// This is for running a potential auto routine but in teleop so you can
					// drive back/reset things easily
					/*
					sfDrive->setArcadeInit(false);
					if (autoState == STATE1) {
						DriverStation::ReportError("State 1");
						if (Timer::GetFPGATimestamp() - lastReceivedTime < 2) {
							autoState = STATE2;
							lastAngle = lastReceived;
						} else {
							autoState = STOP;
						}
					} else if (autoState == STATE2) {
						DriverStation::ReportError("Turning " + std::to_string(lastAngle));
						if (sfDrive->turnToAngle(lastAngle) == true) {
							autoState = STOP;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
						}
					} else if (autoState == STOP) {
						DriverStation::ReportError("Auto turning complete");
					}
					 */

					/*
						if (autoState == STATE1) {
						DriverStation::ReportError("State 1");
						SmartDashboard::PutNumber("State", 1);
						if (sfDrive->driveDistance(116) == true) {
							autoState = STATE2;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
						}

					}
					else if (autoState == STATE2) {
						DriverStation::ReportError("State 2");
						SmartDashboard::PutNumber("State", 2);
						if (sfDrive->turnToAngle(60) == true) {
							autoState = STATE4;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
						}
					}
//					else if (autoState == STATE3) {
//						DriverStation::ReportError("State 3");
//						if (sfDrive->driveDistance(18) == true) {
//							autoState = STATE4;
//							sfDrive->setPIDInit(false);
//							initialTime = Timer::GetFPGATimestamp();
//						}
//						if (Timer::GetFPGATimestamp() > initialTime + 1.5) {
//							autoState = STATE4;
//							sfDrive->setPIDInit(false);
//							initialTime = Timer::GetFPGATimestamp();
//						}
					else if (autoState == STATE4) {
						DriverStation::ReportError("State 4");
						if (Timer::GetFPGATimestamp() > initialTime + 1) {
							autoState = STATE5;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
						}
						lastAngle = lastReceived;
					} else if (autoState == STATE5) {
						DriverStation::ReportError("State 5");
						if (sfDrive->turnToAngle(lastAngle) == true) {
							autoState = STATE6;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
						}
					} else if (autoState == STATE6) {
						DriverStation::ReportError("State 6");
						if (sfDrive->driveDistance(18) == true) {
							autoState = STATE7;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
						}
					} else if (autoState == STOP) {
						DriverStation::ReportError("Auto Complete");
					}
					 */

				} else {
					DriverStation::ReportError("Joystick Drive");

					sfDrive->joystickDrive(joystickMain->GetRawAxis(1), joystickMain->GetRawAxis(4));
					//				versaMotorL->Set(backLID);
					//				versaMotorR->Set(backRID);

					sfDrive->mediumDrive = false;
					sfDrive->setPIDInit(false);
					SmartDashboard::PutNumber("State", 1);
					autoState = STATE1;
				}
			} else {
				teleopInit = false;
			}

			// wait for a motor update time
			frc::Wait(0.005);
		}
	}

	void Test() override {

	}
};

START_ROBOT_CLASS(Robot)
