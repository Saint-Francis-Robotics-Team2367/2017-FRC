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

#define tickRateForward 100
#define tickRateTurn 10
#define wheelDiameter 8

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
	frc::RobotDrive *drive;

	std::vector<double> pidDrive;
	std::vector<double> pidTurn;

	Joystick *joystickMain;

	frc::DigitalInput *gearLimitSwitch;

	AHRS *navX;

	bool autoInit, teleopInit, pidInit;
	int ticksBackL, ticksBackR, totalTicks;
	float initialAngle;
	double initialTime, lastTime;

	int totalCycles;

	frc::SendableChooser<std::string> chooser;
	const std::string autoNothing = "Nothing";
	const std::string autoForward = "DriveForward";
	const std::string autoLowGoal = "LowGoal";
	const std::string autoGear = "Gear";
	const std::string autoHighGoal = "HighGoal";

public:
	Robot() {
		//Note SmartDashboard is not initialized here, wait until RobotInit to make SmartDashboard calls
		//		drive->SetExpiration(0.1);
	}

	void RobotInit() {
		autoInit = false;
		teleopInit = false;
		pidInit = false;

		autoState = STATE1;

		frontMotorL = new CANTalon(1);
		backMotorL = new CANTalon(2);
		frontMotorR = new CANTalon(3);
		backMotorR = new CANTalon(4);

		sfDrive = new SFDrive(frontMotorL, frontMotorR, backMotorL, backMotorR);

		pidDrive = {0.24, 0, 0.18};
		pidTurn = {0.7, 0, 0};

		ticksBackL = 0;
		ticksBackR = 0;
		totalTicks = 0;
		totalCycles = 0;

		drive = new RobotDrive(frontMotorL, backMotorL, frontMotorR, backMotorR);
		joystickMain = new Joystick(0);

		frontMotorL->SetSensorDirection(true);
		frontMotorR->SetSensorDirection(true);
		backMotorL->SetSensorDirection(true);
		backMotorR->SetSensorDirection(true);

		// PORT NEEDS TO BE CHANGED
		gearLimitSwitch = new DigitalInput(0);

		navX = new AHRS(SerialPort::kMXP);

		chooser.AddDefault(autoNothing, autoNothing);
		chooser.AddObject(autoForward, autoForward);
		chooser.AddObject(autoLowGoal, autoLowGoal);
		chooser.AddObject(autoGear, autoGear);
		chooser.AddObject(autoHighGoal, autoHighGoal);
		SmartDashboard::PutData("Auto Modes", &chooser);

	}

	float convertDistanceToTicks (float inches) {
		return inches / wheelDiameter / 3.1415 * 1024 * 4;
	}

	bool turnToAngle (double degreesFromInit, double ticksLeft, double ticksRight, double p, double i, double d) {
		DriverStation::ReportError("Turning");
		// good turning PID's are 0.7, 0, 0
		if (pidInit == false) {
			PIDInit(p, i, d);
			pidInit = true;
		}
		SmartDashboard::PutNumber("Angle to Get", initialAngle + degreesFromInit);

		backMotorL->Set(ticksLeft);
		backMotorR->Set(ticksRight);

		if (abs(abs(navX->GetAngle()) - abs(initialAngle + degreesFromInit)) < 5) {
			DriverStation::ReportError("Done and returning true");
			ticksBackL = -backMotorL->GetEncPosition();
			ticksBackR = -backMotorR->GetEncPosition();
			backMotorL->Set(ticksBackL);
			backMotorR->Set(ticksBackR);
			//			SmartDashboard::PutNumber("Current Setpoint Left", ticksBackL);
			//			SmartDashboard::PutNumber("Current Setpoint Right", ticksBackR);

			return true;
		} else {
			//			resetMotors();

			if (degreesFromInit < 0) {
				ticksBackL -= tickRateTurn;
				ticksBackR -= tickRateTurn;
			} else {
				ticksBackL += tickRateTurn;
				ticksBackR += tickRateTurn;
			}

			return false;
		}
	}
	bool driveDistance (int ticksLeft, int ticksRight, double p, double i, double d) {
		if (pidInit == false) {
			pidInit = true;
			PIDInit(p, i, d);
		}

		//		frontMotorL->Set(x);
		//		frontMotorR->Set(-x);

		backMotorL->Set((totalTicks > 0 ? ticksLeft : -ticksLeft));
		backMotorR->Set((totalTicks > 0 ? -ticksRight : ticksRight));

		// Checks if current tick position is within +-5% of expected total
		if (abs(backMotorL->GetEncPosition()) - abs(ticksLeft) < abs(totalTicks * .05) &&
				abs(backMotorR->GetEncPosition()) - abs(ticksRight) < abs(totalTicks * .05) &&
				abs(ticksLeft) >= abs(totalTicks) && abs(ticksRight) >= abs(totalTicks)) {
			DriverStation::ReportError("PID drive returning true");
			//			backMotorL->Set((totalTicks > 0 ? ticksLeft : -ticksLeft));
			//			backMotorR->Set((totalTicks > 0 ? -ticksRight : ticksRight));
			return true;
		} else {
			if (abs(ticksBackL) < abs(totalTicks)) {
				ticksBackL += tickRateForward;
			}
			if (abs(ticksBackR) < abs(totalTicks)) {
				ticksBackR += tickRateForward;
			}
			return false;
		}
	}

	void PIDInit(double p, double i, double d) {
		DriverStation::ReportError("PID Init");

		navX->Reset();
		initialAngle = 0;

		ticksBackL = 0;
		ticksBackR = 0;
		DriverStation::ReportError("Left Ticks " + std::to_string(ticksBackL));

		frontMotorL->SetEncPosition(0);
		frontMotorR->SetEncPosition(0);
		backMotorL->SetEncPosition(0);
		backMotorR->SetEncPosition(0);

		frontMotorL->SetControlMode(CANSpeedController::kPosition);
		frontMotorR->SetControlMode(CANSpeedController::kPosition);
		backMotorL->SetControlMode(CANSpeedController::kPosition);
		backMotorR->SetControlMode(CANSpeedController::kPosition);

		frontMotorL->Set(0);
		frontMotorR->Set(0);
		backMotorL->Set(0);
		backMotorR->Set(0);

		frontMotorL->SetPID(p, i, d);
		frontMotorR->SetPID(p, i, d);
		backMotorL->SetPID(p, i, d);
		backMotorR->SetPID(p, i, d);

		frontMotorL->SetSensorDirection(true);
		frontMotorR->SetSensorDirection(true);
		backMotorL->SetSensorDirection(true);
		backMotorR->SetSensorDirection(true);

		//		SmartDashboard::PutNumber("Total Ticks", 5000);
		//		SmartDashboard::PutNumber("Current P", 0.1);
		//		SmartDashboard::PutNumber("Current I", 0);
		//		SmartDashboard::PutNumber("Current D", 0.01);
		//		pidDrive[0] = SmartDashboard::GetNumber("Current P", 0);
		//		pidDrive[1] = SmartDashboard::GetNumber("Current I", 0);
		//		pidDrive[2] = SmartDashboard::GetNumber("Current D", 0);

		DriverStation::ReportError(std::to_string(ticksBackR));

		SmartDashboard::PutNumber("Starting ticks", ticksBackR);
	}

	void autonomousInit() {
		//		timeInitial = Timer::GetFPGATimestamp();

		pidInit = false;

		autoState = STATE1;

		frontMotorL->SetEncPosition(0);
		frontMotorR->SetEncPosition(0);
		backMotorL->SetEncPosition(0);
		backMotorR->SetEncPosition(0);

		ticksBackL = 0;
		ticksBackR = 0;

		//		navX->Reset();

		//		initialY = imu->GetAngleY();
		//		initialZ = imu->GetAngleZ() - 12;
		//
		//		timesAuto++;

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
				//				SmartDashboard::PutNumber("AngleX", imu->GetAngleX());
				//				SmartDashboard::PutNumber("AngleY", imu->GetAngleY());
				//				SmartDashboard::PutNumber("AngleZ", imu->GetAngleZ());
				//				SmartDashboard::PutNumber("NavX Angle", navX->GetAngle());
				SmartDashboard::PutNumber("Current Setpoint Left", ticksBackL);
				SmartDashboard::PutNumber("Current Setpoint Right", ticksBackR);
				SmartDashboard::PutNumber("Current P", backMotorL->GetP());
				SmartDashboard::PutNumber("Current I", backMotorL->GetI());
				SmartDashboard::PutNumber("Current D", backMotorL->GetD());
				SmartDashboard::PutNumber("Left Front", frontMotorL->GetEncPosition());
				SmartDashboard::PutNumber("Right Front", frontMotorR->GetEncPosition());
				SmartDashboard::PutNumber("Left Back", backMotorL->GetEncPosition());
				SmartDashboard::PutNumber("Right Back", backMotorR->GetEncPosition());

				std::string ticksAndTime = std::to_string(backMotorL->GetEncPosition()) + " "+ std::to_string(Timer::GetFPGATimestamp());
				DriverStation::ReportError(ticksAndTime);

				if (autoSelected == autoNothing) {
					if (autoState == STATE1) {
						DriverStation::ReportError("State 1");

						totalTicks = convertDistanceToTicks(96);
						SmartDashboard::PutNumber("Ticks to Move", convertDistanceToTicks(96));
						if (driveDistance(
								ticksBackL,
								ticksBackR,
								pidDrive[0],
								pidDrive[1],
								pidDrive[2]) == true) {
							// finished driving distance
							autoState = STATE2;
							pidInit = false;
							DriverStation::ReportError("Finished forward");

							resetMotors();

							//		frontMotorL->Set(x);
							//		frontMotorR->Set(-x);
							backMotorL->Set(backMotorL->GetEncPosition());
							backMotorR->Set(backMotorR->GetEncPosition());
						} else {
						}
					} else if (autoState == STATE2) {
						DriverStation::ReportError("State 2");
						if (turnToAngle(SmartDashboard::GetNumber("Degrees to Turn", 0),
								ticksBackL,
								ticksBackR,
								pidTurn[0],
								pidTurn[1],
								pidTurn[2]) == true) {

							autoState = STATE3;
							pidInit = false;
							DriverStation::ReportError("Finished turning");
							ticksBackL = -backMotorL->GetEncPosition();
							ticksBackR = -backMotorR->GetEncPosition();
							backMotorL->Set(ticksBackL);
							backMotorR->Set(ticksBackR);
						} else {
						}
					} else if (autoState == STATE3) {
						DriverStation::ReportError("State 3");

						totalTicks = convertDistanceToTicks(56);
						SmartDashboard::PutNumber("Ticks to Move", convertDistanceToTicks(56));
						if (driveDistance(
								ticksBackL,
								ticksBackR,
								pidDrive[0],
								pidDrive[1],
								pidDrive[2]) == true) {
							// finished driving distance
							autoState = STATE4;
							pidInit = false;
							DriverStation::ReportError("Finished forward");

							resetMotors();

							//		frontMotorL->Set(x);
							//		frontMotorR->Set(-x);
							backMotorL->Set(backMotorL->GetEncPosition());
							backMotorR->Set(backMotorR->GetEncPosition());
						} else {
						}
					}
				} else if (autoSelected == autoForward) {
					//					if (pidInit == false) {
					//						pidInit = true;
					//						PIDInit(SmartDashboard::GetNumber("Left P", 0),
					//								SmartDashboard::GetNumber("Left I", 0),
					//								SmartDashboard::GetNumber("Left D", 0));
					//					}
					//
					//					double timeDiff = Timer::GetFPGATimestamp() - lastTime;
					//					double ticksIncrement = 3 * 4096 * timeDiff;
					//
					//					(ticksBackL > 0 ? ticksBackL += ticksIncrement : ticksBackL -= ticksIncrement);
					//					(ticksBackR > 0 ? ticksBackR += ticksIncrement : ticksBackR -= ticksIncrement);
					//
					//					backMotorL->Set((ticksToMove > 0 ? ticksBackL : -ticksBackL));
					//					backMotorR->Set((ticksToMove > 0 ? -ticksBackR : ticksBackR));
					//
					//					lastTime = Timer::GetFPGATimestamp();
				} else if (autoSelected == autoLowGoal) {
				} else if (autoSelected == autoGear) {
				} else if (autoSelected == autoHighGoal) {
				}
			} else {
				autoInit = false;
			}
		}
	}

	void teleoperatedInit() {
		SmartDashboard::PutData("Auto Modes", &chooser);

		DriverStation::ReportError("Teleop Init");

		navX->Reset();

		frontMotorL->SetEncPosition(0);
		frontMotorR->SetEncPosition(0);
		backMotorL->SetEncPosition(0);
		backMotorR->SetEncPosition(0);

		frontMotorL->SetControlMode(CANSpeedController::kPercentVbus);
		frontMotorR->SetControlMode(CANSpeedController::kPercentVbus);
		backMotorL->SetControlMode(CANSpeedController::kPercentVbus);
		backMotorR->SetControlMode(CANSpeedController::kPercentVbus);

		ticksBackL = 0;
		ticksBackR = 0;

		initialTime = Timer::GetFPGATimestamp();

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
				SmartDashboard::PutNumber("Setpoint 1", ticksBackL);
				SmartDashboard::PutNumber("Error 1", abs(ticksBackL) - abs(backMotorL->GetEncPosition()));

				// just to avoid motor safety helpers if not called later, although it should be
				//				drive->ArcadeDrive(0.0, 0);

				if (gearLimitSwitch->Get() == 0) {
					// Switch Closed - possessing gear
				} else if (gearLimitSwitch->Get() == 1) {
					// Switch Open - not possessing gear
				} else {
					DriverStation::ReportError("Error: Limit switch returning something not 0 or 1");
				}

				/*
				if (joystickMain->GetRawButton(2)) {
					if (pidInit == false) {
						DriverStation::ReportError("ACTUALLY INITIALIZING");
						pidInit = true;

						PIDInit(pidTurn[0], pidTurn[1], pidTurn[2]);

						// Doing this here too because it takes a while to actually finish
						ticksBackL = 0;
						ticksBackR = 0;
						frontMotorL->SetEncPosition(0);
						frontMotorR->SetEncPosition(0);
						backMotorL->SetEncPosition(0);
						backMotorR->SetEncPosition(0);
						backMotorL->Set(0);
						backMotorR->Set(0);

						SmartDashboard::PutNumber("Setpoint 1", ticksBackL);
						SmartDashboard::PutNumber("Error 1", abs(ticksBackL) - abs(backMotorL->GetEncPosition()));
					}
					//					DriverStation::ReportError("State 2");
					if (turnToAngle(SmartDashboard::GetNumber("Degrees to Turn", 0),
							ticksBackL,
							ticksBackR,
							pidTurn[0],
							pidTurn[1],
							pidTurn[2]) == true) {

						autoState = STATE3;
						DriverStation::ReportError("Finished turning");
						//						ticksBackL = -backMotorL->GetEncPosition();
						//						ticksBackR = -backMotorR->GetEncPosition();

						frontMotorL->SetEncPosition(0);
						frontMotorR->SetEncPosition(0);
						backMotorL->SetEncPosition(0);
						backMotorR->SetEncPosition(0);
						backMotorL->Set(0);
						backMotorR->Set(0);
					} else {
					}
				} else {
					pidInit = false;
				}
*/

				if (joystickMain->GetRawButton(1)) {
					if (autoState == STATE1) {
						if (pidInit == false) {
							pidInit = true;

							totalCycles++;
							pidDrive[0] = SmartDashboard::GetNumber("Current P", 0);
							pidDrive[1] = SmartDashboard::GetNumber("Current I", 0);
							pidDrive[2] = SmartDashboard::GetNumber("Current D", 0);
							PIDInit(pidDrive[0], pidDrive[1], pidDrive[2]);

							//	ticksString = "Ticks" + std::to_string(totalCycles) + " " + std::to_string(currentP) + "-"
							//	+ std::to_string(currentI) + "-" + std::to_string(currentD);
							//	timeString = "Time " + std::to_string(totalCycles);

							// For setting drive distance from SmartDashboard
							/*
							if (SmartDashboard::GetNumber("Total Ticks", 0) < 500) {
								if (totalTicks > 0)
									totalTicks = -1 * convertDistanceToTicks(SmartDashboard::GetNumber("Total Ticks", 0));
								else
									totalTicks = convertDistanceToTicks(SmartDashboard::GetNumber("Total Ticks", 0));
								SmartDashboard::PutNumber("Total Ticks", convertDistanceToTicks(SmartDashboard::GetNumber("Total Ticks", 0)));
							} else {
								if (totalTicks > 0)
									totalTicks = -1 * SmartDashboard::GetNumber("Total Ticks", 0);
								else
									totalTicks = SmartDashboard::GetNumber("Total Ticks", 0);
							}
							*/
						}
						if (driveDistance(
								ticksBackL,
								ticksBackR,
								pidDrive[0],
								pidDrive[1],
								pidDrive[2]) == true) {

							// Finished driving distance
							DriverStation::ReportError("Finished forward");

							//		frontMotorL->Set(x);
							//		frontMotorR->Set(-x);
							//						backMotorL->Set(backMotorL->GetEncPosition());
							//						backMotorR->Set(backMotorR->GetEncPosition());
						} else {
							DriverStation::ReportError("Still driving");
						}
					} else if (autoState == STATE2) {

					} else if (autoState == STATE3) {

					} else if (autoState == STATE4) {

					}
				} else {
					pidInit = false;
				}
				//				drive->ArcadeDrive(-joystickMain->GetRawAxis(1), -joystickMain->GetRawAxis(4));

			} else {
				teleopInit = false;
			}

			// wait for a motor update time
			frc::Wait(0.005);
		}
	}

	void Test() override {

	}

	void resetMotors() {
		if (drive->GetGlobalError().GetCode() != 0) {
			drive->GetGlobalError().GetOriginatingObject()->ClearError();

			frontMotorL->ClearError();
			frontMotorR->ClearError();
			backMotorL->ClearError();
			backMotorR->ClearError();
		}

		drive->GetGlobalError().Clear();
		frontMotorL->EnableControl();
		frontMotorR->EnableControl();
		backMotorL->EnableControl();
		backMotorR->EnableControl();
	}
};

START_ROBOT_CLASS(Robot)
