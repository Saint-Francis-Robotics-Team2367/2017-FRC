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

	bool autoInit, teleopInit;
	double initialTime;

	Joystick *joystickMain;

	frc::DigitalInput *gearLimitSwitch;

	AHRS *navX;

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

		autoState = STATE1;

		frontMotorL = new CANTalon(1);
		backMotorL = new CANTalon(2);
		frontMotorR = new CANTalon(3);
		backMotorR = new CANTalon(4);

		navX = new AHRS(SerialPort::kMXP);

		sfDrive = new SFDrive(frontMotorL, frontMotorR, backMotorL, backMotorR);
		sfDrive->navX = this->navX;

		joystickMain = new Joystick(0);

		// PORT NEEDS TO BE CHANGED
		gearLimitSwitch = new DigitalInput(0);

		chooser.AddDefault(autoNothing, autoNothing);
		chooser.AddObject(autoForward, autoForward);
		chooser.AddObject(autoLowGoal, autoLowGoal);
		chooser.AddObject(autoGear, autoGear);
		chooser.AddObject(autoHighGoal, autoHighGoal);
		SmartDashboard::PutData("Auto Modes", &chooser);

	}

	void autonomousInit() {
		//		timeInitial = Timer::GetFPGATimestamp();

		autoState = STATE1;

		frontMotorL->SetEncPosition(0);
		frontMotorR->SetEncPosition(0);
		backMotorL->SetEncPosition(0);
		backMotorR->SetEncPosition(0);

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
				SmartDashboard::PutNumber("Current Setpoint Left", sfDrive->getSetpointLeft());
				SmartDashboard::PutNumber("Current Setpoint Right", sfDrive->getSetpointRight());
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

						//						totalTicks = convertDistanceToTicks(96);
						SmartDashboard::PutNumber("Ticks to Move", sfDrive->convertDistanceToTicks(96));

					} else if (autoState == STATE2) {
						DriverStation::ReportError("State 2");

					} else if (autoState == STATE3) {
						DriverStation::ReportError("State 3");

						//						totalTicks = convertDistanceToTicks(56);
						SmartDashboard::PutNumber("Ticks to Move", sfDrive->convertDistanceToTicks(56));

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

		autoState = STATE1;

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
				SmartDashboard::PutNumber("Ticks 1", backMotorL->GetEncPosition());
				//				SmartDashboard::PutNumber("SetpointLeft", ticksBackL);
				SmartDashboard::PutNumber("Setpoint 1", sfDrive->getSetpointLeft());
				SmartDashboard::PutNumber("Error 1", abs(sfDrive->getSetpointLeft()) - abs(backMotorL->GetEncPosition()));

				if (gearLimitSwitch->Get() == 0) {
					// Switch Closed - possessing gear
				} else if (gearLimitSwitch->Get() == 1) {
					// Switch Open - not possessing gear
				} else {
					DriverStation::ReportError("Error: Limit switch returning something not 0 or 1");
				}

				if (joystickMain->GetRawButton(1)) {

					sfDrive->setArcadeInit(false);

					if (autoState == STATE1) {
						DriverStation::ReportError("State 1");

						if (Timer::GetFPGATimestamp() < initialTime + 1) {
						} else if (sfDrive->driveDistance(96) == true) {
							autoState = STATE2;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
						}
					} else if (autoState == STATE2) {
						DriverStation::ReportError("State 2");

						if (Timer::GetFPGATimestamp() < initialTime + 1) {
						} else if (sfDrive->turnToAngle(180) == true) {
							autoState = STATE1;
							sfDrive->setPIDInit(false);
							initialTime = Timer::GetFPGATimestamp();
						}
					}
				} else {
					DriverStation::ReportError("Joystick Drive");
					sfDrive->joystickDrive(-joystickMain->GetRawAxis(1), -joystickMain->GetRawAxis(4));
					sfDrive->setPIDInit(false);
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
