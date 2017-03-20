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

#define tickRateForward 100
#define tickRateTurn 10
#define wheelDiameter 8

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
 */

#define versaLID 7
#define frontLID 2
#define backLID 1
#define frontRID 3
#define backRID 4
#define versaRID 5
#define climberID 6

class Robot: public frc::SampleRobot {

	enum AutoState {
		STATE1,
		STATE2,
		STATE3,
		STATE4,
		STATE5,
		STATE6,
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

	int lastDistance, lastAngle;

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

		rampSolenoid = new DoubleSolenoid(2, 3);
		dropOffSolenoid = new DoubleSolenoid(0, 1);
		climberSolenoid = new DoubleSolenoid(4, 5);

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
		SmartDashboard::PutData("Auto Modes", &chooser);

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

		SmartDashboard::PutData("Auto Modes", &chooser);

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
				SmartDashboard::PutNumber("Current P", backMotorL->GetP());
				SmartDashboard::PutNumber("Current I", backMotorL->GetI());
				SmartDashboard::PutNumber("Current D", backMotorL->GetD());
				SmartDashboard::PutNumber("Left Back", backMotorL->GetEncPosition());
				SmartDashboard::PutNumber("Right Back", backMotorR->GetEncPosition());

				//				std::string ticksAndTime = std::to_string(backMotorL->GetEncPosition()) + " "+ std::to_string(Timer::GetFPGATimestamp());
				//				DriverStation::ReportError(ticksAndTime);

				if (autoSelected == autoNothing) {

				} else if (autoSelected == autoForward) {
					if (forwardCycles < 4) {
						if (autoState == STATE1) {
							DriverStation::ReportError("State 1");
							if (Timer::GetFPGATimestamp() < initialTime + 1.5) {
								sfDrive->joystickDrive(1.0, 0);
								//							versaMotorL->Set(backLID);
								//							versaMotorR->Set(backRID);
							} else {
								autoState = STATE2;
								initialTime = Timer::GetFPGATimestamp();
								forwardCycles++;
							}
						} else if (autoState == STATE2) {
							DriverStation::ReportError("State 2");
							if (Timer::GetFPGATimestamp() < initialTime + 1) {
								// delay between bursts
							} else {
								autoState = STATE1;
							}
						}
					}
					/*
					if (autoState == STATE1) {
						DriverStation::ReportError("State 1");
						if (sfDrive->driveDistance(108) == true) {
							autoState = STATE2;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
						}
					} else if (autoState == STATE2) {

					}
					 */
				} else if (autoSelected == autoGearF) {
					if (autoState == STATE1) {
						DriverStation::ReportError("State 1: driving 50");
						if (sfDrive->driveDistance(35) == true) {
							autoState = STATE2;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
							lastDistanceExecuted = abs(backMotorL->GetEncPosition());

							double error = abs(autoStartingAngle) - abs(navX->GetAngle());
							if (error < 4)
								autoState = STATE3;
							else {
								autoState = STATE2;
								angleToTurn = autoStartingAngle - navX->GetAngle();
							}
						}
					} else if (autoState == STATE2) {
						DriverStation::ReportError("State 2: turning " + std::to_string(angleToTurn));
						if (sfDrive->turnToAngle(angleToTurn) == true) {
							autoState = STATE3;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
						}
					} else if (autoState == STATE3) {
						DriverStation::ReportError("State 3: driving " + std::to_string(sfDrive->convertDistanceToTicks(80) - lastDistanceExecuted));
						if (sfDrive->driveDistance(sfDrive->convertDistanceToTicks(61) - lastDistanceExecuted) == true) {
							autoState = STATE2;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
							lastDistanceExecuted = abs(backMotorL->GetEncPosition());

							double error = abs(autoStartingAngle) - abs(navX->GetAngle());
							if (error < 4)
								autoState = STATE5;
							else {
								autoState = STATE4;
								angleToTurn = autoStartingAngle - navX->GetAngle();
							}
						}
					} else if (autoState == STATE6) {

					}
				} else if (autoSelected == autoGearL) {

				} else if (autoSelected == autoGearR) {
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

		autoState = STATE1;
		timeDrop = 0;
		debounceTime = 0;

		frontMotorL->SetControlMode(CANSpeedController::kPercentVbus);
		frontMotorR->SetControlMode(CANSpeedController::kPercentVbus);
		backMotorL->SetControlMode(CANSpeedController::kPercentVbus);
		backMotorR->SetControlMode(CANSpeedController::kPercentVbus);
		versaMotorL->SetControlMode(CANSpeedController::kFollower);
		versaMotorR->SetControlMode(CANSpeedController::kFollower);

		//		backMotorL->SetControlMode(CANSpeedController::kSpeed);
		//		backMotorL->SetSensorDirection(true);
		//		backMotorL->SetPID(SmartDashboard::GetNumber("Current P", 0),
		//				SmartDashboard::GetNumber("Current I", 0),
		//				SmartDashboard::GetNumber("Current D", 0));
		//		SmartDashboard::PutNumber("SetRPM", 0);
		//		DriverStation::ReportError("Putting");

		SmartDashboard::PutData("Auto Modes", &chooser);

		//		SmartDashboard::PutNumber("AngleToTurn2", 0);
	}
	void OperatorControl() override {
		//		drive->SetSafetyEnabled(true);
		while (IsOperatorControl()) {
			if (IsEnabled()) {
				if (!teleopInit) {
					teleopInit = true;
					teleoperatedInit();
				}
				//				SmartDashboard::PutNumber("AngleX", imu->GetAngleX());
				//				SmartDashboard::PutNumber("AngleY", imu->GetAngleY());
				//				SmartDashboard::PutNumber("AngleZ", imu->GetAngleZ());
				SmartDashboard::PutNumber("NavX Angle", navX->GetAngle());
				SmartDashboard::PutNumber("Ticks 1", backMotorL->GetEncPosition());
				//				SmartDashboard::PutNumber("SetpointLeft", ticksBackL);
				SmartDashboard::PutNumber("Setpoint 1", sfDrive->getSetpointLeft());
				SmartDashboard::PutNumber("Total Ticks", sfDrive->totalTicks);
				SmartDashboard::PutNumber("Error 1", abs(sfDrive->getSetpointLeft()) - abs(backMotorL->GetEncPosition()));
				//				SmartDashboard::PutNumber("LastReceived", lastAngle);

				if (dropOffSolenoid->Get() == DoubleSolenoid::kForward)
					SmartDashboard::PutString("Drop Off", "Down");
				else
					SmartDashboard::PutString("Drop Off", "Up");
				if (rampSolenoid->Get() == DoubleSolenoid::kForward)
					SmartDashboard::PutString("Ramp", "Down");
				else
					SmartDashboard::PutString("Ramp", "Up");

				/*
//				if (gearLimitSwitch->Get() == 0) {
//					// Switch Closed - possessing gear
//				} else if (gearLimitSwitch->Get() == 1) {
//					// Switch Open - not possessing gear
//				} else {
//					DriverStation::ReportError("Error: Limit switch returning something not 0 or 1");
//				}

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



				//				if (Timer::GetFPGATimestamp() > timeDrop + 0.5 && debounceTime == -1) {
				//					rampSolenoid->Set(DoubleSolenoid::kForward);
				//				}

				if (Timer::GetFPGATimestamp() > debounceTime + 0.5 && Timer::GetFPGATimestamp() > timeDrop + 0.5) {
					rampSolenoid->Set(DoubleSolenoid::kForward);
				}
				if (joystickMain->GetRawButton(1) || joystickSecond->GetRawButton(1)) {
					timeDrop = Timer::GetFPGATimestamp();
					if (rampSolenoid->Get() == DoubleSolenoid::kForward) {
						rampSolenoid->Set(DoubleSolenoid::kReverse);
					}
				}
				if (joystickMain->GetRawButton(4) || joystickSecond->GetRawButton(4)) {
					if (rampSolenoid->Get() == DoubleSolenoid::kForward) {
						debounceTime = Timer::GetFPGATimestamp();
						rampSolenoid->Set(DoubleSolenoid::kReverse);
					}
				}

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


				if (joystickMain->GetRawButton(1)) {

					sfDrive->setArcadeInit(false);
					if (autoState == STATE1) {
						DriverStation::ReportError("State 1");
						if (sfDrive->turnToAngle(SmartDashboard::GetNumber("LastReceived", 0)) == true) {
							autoState = STATE2;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
						}
					} else if (autoState == STATE2) {
						DriverStation::ReportError("State 2");

					}

					//					if (autoState == STATE1) {
					//						DriverStation::ReportError("State 1");
					//						if (sfDrive->driveDistance(90.6)) {
					//							autoState = STATE2;
					//							sfDrive->setPIDInit(false);
					//							initialTime = Timer::GetFPGATimestamp();
					//							SmartDashboard::PutNumber("TicksDriven", backMotorL->GetEncPosition());
					//						}
					//					} else if (autoState == STATE2) {
					//						DriverStation::ReportError("State 2");
					//						if (sfDrive->turnToAngle(SmartDashboard::GetNumber("LastReceived", 0)) == true) {
					//							autoState = STATE3;
					//							sfDrive->setPIDInit(false);
					//							initialTime = Timer::GetFPGATimestamp();
					//						}
					//					}

					//					else if (autoState == STATE2) {
					//						DriverStation::ReportError("State 2");
					//
					//						if (Timer::GetFPGATimestamp() < initialTime + 1) {
					//						} else if (sfDrive->driveDistance(SmartDashboard::GetNumber("Current Setpoint Left", 0)) == true) {
					//							autoState = STATE3;
					//							sfDrive->setPIDInit(false);
					//							initialTime = Timer::GetFPGATimestamp();
					//						}
					//						//						if (Timer::GetFPGATimestamp() < initialTime + 1) {
					//						//						} else if (sfDrive->turnToAngle(SmartDashboard::GetNumber("AngleToTurn2", 0)) == true) {
					//						//							autoState = STATE3;
					//						//							sfDrive->setPIDInit(false);
					//						//							initialTime = Timer::GetFPGATimestamp();
					//					}
				} else {
					DriverStation::ReportError("Joystick Drive");
					//					backMotorL->Set(SmartDashboard::GetNumber("SetRPM", 0));
					//					SmartDashboard::PutNumber("GetRPM", backMotorL->GetSpeed());
					//					SmartDashboard::PutNumber("Current", backMotorL->GetOutputVoltage());

					sfDrive->joystickDrive(-joystickMain->GetRawAxis(1), (-joystickMain->GetRawAxis(1) * driveTurnConstant) -joystickMain->GetRawAxis(4));
					//				versaMotorL->Set(backLID);
					//				versaMotorR->Set(backRID);

					sfDrive->setPIDInit(false);
					autoState = STATE1;

					if (visionHelper->udpSocket->hasPendingPacket()) {
						visionHelper->receivePendingUDP();
						string udpReceive = visionHelper->udpReceiveString;
						//						lastAngle = std::stoi(udpReceive);
						DriverStation::ReportError("Received " + udpReceive + " from " + visionHelper->hostIP);
					} else {
						//						lastAngle = SmartDashboard::GetNumber("LastReceived", 0);
						DriverStation::ReportError("Receiving nothing");
					}
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
