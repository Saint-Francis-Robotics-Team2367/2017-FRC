#include <iostream>
#include <memory>
#include <string>

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

#define tickRateForward 100
#define tickRateTurn 35
#define wheelDiameter 8

/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will
 * automatically call your Autonomous and OperatorControl methods at the right
 * time as controlled by the switches on the driver station or the field
 * controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
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

	CANTalon *frontMotorL;
	CANTalon *frontMotorR;
	CANTalon *backMotorL;
	CANTalon *backMotorR;
	frc::RobotDrive *drive;

	Joystick *joystickMain;

	frc::DigitalInput *gearLimitSwitch;

	AHRS *navX;

	bool autoInit, teleopInit, pidInit;
	int ticksBackL, ticksBackR;
	float initialAngle;

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

		ticksBackL = 0;
		ticksBackR = 0;

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

		SmartDashboard::PutNumber("Inches to Move", 96);
	}

	float convertDistanceToTicks (float inches) {
		return inches / wheelDiameter / 3.1415 * 1024 * 4;
	}

	bool turnToAngle (float degreesFromInit, float ticksLeft, float ticksRight, float p, float i, float d) {
		DriverStation::ReportError("Turning");
		// good turning PID's are 0.7, 0, 0
		if (pidInit == false) {
			PIDInit(p, i, d);
			pidInit = true;
		}
		SmartDashboard::PutNumber("Angle to Get", initialAngle + degreesFromInit);

		if (abs(navX->GetAngle() - (initialAngle + degreesFromInit)) < 5) {
			DriverStation::ReportError("Done and returning true");
			ticksBackL = -backMotorL->GetEncPosition();
			ticksBackR = -backMotorR->GetEncPosition();
			backMotorL->Set(ticksBackL);
			backMotorR->Set(ticksBackR);
			SmartDashboard::PutNumber("Current Setpoint Left", ticksBackL);
			SmartDashboard::PutNumber("Current Setpoint Right", ticksBackR);

			return true;
		} else {
			//			resetMotors();
			backMotorL->Set(ticksLeft);
			backMotorR->Set(ticksRight);

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
	bool driveDistance (float ticksLeft, float ticksRight, float p, float i, float d) {
		if (pidInit == false) {
			pidInit = true;
			PIDInit(p, i, d);
		}

		//		frontMotorL->Set(x);
		//		frontMotorR->Set(-x);
		backMotorL->Set((SmartDashboard::GetNumber("Ticks to Move", 0) > 0 ? ticksLeft : -ticksLeft));
		backMotorR->Set((SmartDashboard::GetNumber("Ticks to Move", 0) > 0 ? -ticksRight : ticksRight));
		if (abs(abs(backMotorL->GetEncPosition()) - ticksLeft) < 300 &&
				abs(abs(backMotorR->GetEncPosition()) - ticksRight) < 300) {
			backMotorL->Set(backMotorL->GetEncPosition());
			backMotorR->Set(backMotorR->GetEncPosition());
			return true;
		} else {
			double ticksToMove = SmartDashboard::GetNumber("Ticks to Move", 0);
			if (abs(ticksBackL) < abs(ticksToMove)) {
				ticksBackL += tickRateForward;
			}
			if (abs(ticksBackR) < abs(ticksToMove)) {
				ticksBackR += tickRateForward;
			}
			return false;
		}
	}

	void PIDInit(float p, float i, float d) {
		DriverStation::ReportError("PID Init");

		navX->Reset();

		ticksBackL = tickRateForward + 200;
		ticksBackR = tickRateForward + 200;

		initialAngle = 0;
		SmartDashboard::PutNumber("Initial Angle", initialAngle);

		frontMotorL->SetEncPosition(0);
		frontMotorR->SetEncPosition(0);
		backMotorL->SetEncPosition(0);
		backMotorR->SetEncPosition(0);

		frontMotorL->SetPID(p, i, d);
		frontMotorR->SetPID(p, i, d);
		backMotorL->SetPID(p, i, d);
		backMotorR->SetPID(p, i, d);

		frontMotorL->SetSensorDirection(true);
		frontMotorR->SetSensorDirection(true);
		backMotorL->SetSensorDirection(true);
		backMotorR->SetSensorDirection(true);

		frontMotorL->SetControlMode(CANSpeedController::kPosition);
		frontMotorR->SetControlMode(CANSpeedController::kPosition);
		backMotorL->SetControlMode(CANSpeedController::kPosition);
		backMotorR->SetControlMode(CANSpeedController::kPosition);

		frontMotorL->Set(0);
		frontMotorR->Set(0);
		backMotorL->Set(0);
		backMotorR->Set(0);
	}

	void autonomousInit() {
		//		timeInitial = Timer::GetFPGATimestamp();

		pidInit = false;

		autoState = STATE1;

		frontMotorL->SetEncPosition(0);
		frontMotorR->SetEncPosition(0);
		backMotorL->SetEncPosition(0);
		backMotorR->SetEncPosition(0);

		ticksBackL = tickRateForward;
		ticksBackR = tickRateForward;

		navX->Reset();

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
				SmartDashboard::PutNumber("NavX Angle", navX->GetAngle());
				SmartDashboard::PutNumber("Current Setpoint Left", ticksBackL);
				SmartDashboard::PutNumber("Current Setpoint Right", ticksBackR);
				SmartDashboard::PutNumber("Current P", backMotorL->GetP());
				SmartDashboard::PutNumber("Current I", backMotorL->GetI());
				SmartDashboard::PutNumber("Current D", backMotorL->GetD());
				SmartDashboard::PutNumber("Left Front", frontMotorL->GetEncPosition());
				SmartDashboard::PutNumber("Right Front", frontMotorR->GetEncPosition());
				SmartDashboard::PutNumber("Left Back", backMotorL->GetEncPosition());
				SmartDashboard::PutNumber("Right Back", backMotorR->GetEncPosition());

				if (autoSelected == autoNothing) {
					if (autoState == STATE1) {
						DriverStation::ReportError("State 1");

						SmartDashboard::PutNumber("Ticks to Move", convertDistanceToTicks(96));
						if (driveDistance(
								ticksBackL,
								ticksBackR,
								SmartDashboard::GetNumber("Left P", 0),
								SmartDashboard::GetNumber("Left I", 0),
								SmartDashboard::GetNumber("Left D", 0)) == true) {
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
								0.7, 0, 0) == true) {

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

						SmartDashboard::PutNumber("Ticks to Move", convertDistanceToTicks(56));
						if (driveDistance(
								ticksBackL,
								ticksBackR,
								SmartDashboard::GetNumber("Left P", 0),
								SmartDashboard::GetNumber("Left I", 0),
								SmartDashboard::GetNumber("Left D", 0)) == true) {
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
				SmartDashboard::PutNumber("Current P", frontMotorL->GetP());
				SmartDashboard::PutNumber("Current I",frontMotorL->GetI());
				SmartDashboard::PutNumber("Current D", frontMotorL->GetD());
				SmartDashboard::PutNumber("Left Front", frontMotorL->GetEncPosition());
				SmartDashboard::PutNumber("Right Front", frontMotorR->GetEncPosition());
				SmartDashboard::PutNumber("Left Back", backMotorL->GetEncPosition());
				SmartDashboard::PutNumber("Right Back", backMotorR->GetEncPosition());
				if (gearLimitSwitch->Get() == 0) {
					// Switch Closed - possessing gear
				} else if (gearLimitSwitch->Get() == 1) {
					// Switch Open - not possessing gear
				} else {
					DriverStation::ReportError("Error: Limit switch returning something not 0 or 1");
				}
				drive->ArcadeDrive(-joystickMain->GetRawAxis(1), -joystickMain->GetRawAxis(4));
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
