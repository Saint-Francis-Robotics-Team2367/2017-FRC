/*
 * SFDrive.cpp
 *
 *  Created on: Feb 6, 2017
 *      Author: Sameer Vijay
 */

#include <SFDrive.h>
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

/*
#define tickRateForwardBig 325
#define tickRateForwardMedium 175
#define tickRateForwardSmall 325
*/
#define tickRateForwardBig 200
#define tickRateForwardMedium 175
#define tickRateForwardSmall 200
// 100 is okay for large turns; smaller turns need smaller tick rates
// 35 is good for small angles
#define tickRateTurnBig 35
#define tickRateTurnSmall 35
#define wheelDiameter 4

SFDrive::SFDrive(CANTalon *frontLeft, CANTalon *frontRight, CANTalon *backLeft, CANTalon *backRight) {
	frontMotorL = frontLeft;
	backMotorL = backLeft;
	frontMotorR = frontRight;
	backMotorR = backRight;

	drive = new RobotDrive(frontMotorL, backMotorL, frontMotorR, backMotorR);

	// This gets actually initialized after SFDrive's constructor is called in whatever class the sfDrive
	// object is created. I wanted to avoid needing a navX parameter in the constructor so SFDrive can be used
	// without a navX, and this is the easiest away
	navX = NULL;

	arcadeInit = false;
	pidInit = false;

	ticksBackL = 0;
	ticksBackR = 0;
	totalTicks = 0;
	initialAngle = 0;
	totalCycles = 0;

	//	pidDriveVals = {0.24, 0, 0.18};
	//	pidTurnVals = {0.7, 0, 0};

	pidDriveVals = {0.5, 0, 0.4};
	pidTurnVals = {0.4, 0, 0};

	mediumDrive = false;
}

bool SFDrive::driveDistance(double distance) {
	if (pidInit == false) {
		pidInit = true;
		if (distance < 500) {
			totalTicks = -convertDistanceToTicks(distance);
		} else {
			totalTicks = -distance;
		}
		SmartDashboard::PutNumber("Total Ticks", totalTicks);

		totalCycles++;
		//		pidDrive[0] = SmartDashboard::GetNumber("Current P", 0);
		//		pidDrive[1] = SmartDashboard::GetNumber("Current I", 0);
		//		pidDrive[2] = SmartDashboard::GetNumber("Current D", 0);
		PIDInit(pidDriveVals);
	}
	if (pidDrive(ticksBackL, ticksBackR, pidDriveVals) == true) {
		// Finished driving distance
		DriverStation::ReportError("Finished driving");

		// Rezeroes everything when done
		frontMotorL->SetEncPosition(0);
		frontMotorR->SetEncPosition(0);
		backMotorL->SetEncPosition(0);
		backMotorR->SetEncPosition(0);
		frontMotorL->Set(0);
		frontMotorR->Set(0);
		backMotorL->Set(0);
		backMotorR->Set(0);

		return true;
	} else {
		DriverStation::ReportError("Still driving");

		return false;
	}
}
bool SFDrive::turnToAngle(double angle) {
	if (pidInit == false) {
		pidInit = true;

		// Doing this here too because it takes a while to actually finish
		ticksBackL = 0;
		ticksBackR = 0;
		frontMotorL->SetEncPosition(0);
		frontMotorR->SetEncPosition(0);
		backMotorL->SetEncPosition(0);
		backMotorR->SetEncPosition(0);
		frontMotorL->Set(0);
		frontMotorR->Set(0);
		backMotorL->Set(0);
		backMotorR->Set(0);

		PIDInit(pidTurnVals);
	}
	if (pidTurn(angle, ticksBackL, ticksBackR, pidTurnVals) == true) {

		DriverStation::ReportError("Finished turning");

		// Rezeroes everything when done
		//		frontMotorL->SetEncPosition(0);
		//		frontMotorR->SetEncPosition(0);
		//		backMotorL->SetEncPosition(0);
		//		backMotorR->SetEncPosition(0);
		//		frontMotorL->Set(frontMotorL->GetEncPosition());
		//		frontMotorR->Set(frontMotorR->GetEncPosition());
		//		backMotorL->Set(backMotorL->GetEncPosition());
		//		backMotorR->Set(backMotorR->GetEncPosition());

		return true;
	} else {
		DriverStation::ReportError("Still turning");

		return false;
	}
}
void SFDrive::joystickDrive(double forwardValue, double rotationValue) {
	if (arcadeInit == false) {
		ArcadeInit();
		arcadeInit = true;
	}
	drive->ArcadeDrive(forwardValue, rotationValue);
}

void SFDrive::ArcadeInit() {
	DriverStation::ReportError("Arcade Init");
//	frontMotorL->SetEncPosition(0);
//	frontMotorR->SetEncPosition(0);
//	backMotorL->SetEncPosition(0);
//	backMotorR->SetEncPosition(0);

	frontMotorL->SetControlMode(CANSpeedController::kPercentVbus);
	frontMotorR->SetControlMode(CANSpeedController::kPercentVbus);
	backMotorL->SetControlMode(CANSpeedController::kPercentVbus);
	backMotorR->SetControlMode(CANSpeedController::kPercentVbus);
}
void SFDrive::PIDInit(std::vector<double> pidConstants) {
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

	//#warning "REMOVE THIS"
	//	pidConstants[0] = SmartDashboard::GetNumber("Current P", 0);
	//	pidConstants[1] = SmartDashboard::GetNumber("Current I", 0);
	//	pidConstants[2] = SmartDashboard::GetNumber("Current D", 0);
	frontMotorL->SetPID(pidConstants[0], pidConstants[1], pidConstants[2]);
	frontMotorR->SetPID(pidConstants[0], pidConstants[1], pidConstants[2]);
	backMotorL->SetPID(pidConstants[0], pidConstants[1], pidConstants[2]);
	backMotorR->SetPID(pidConstants[0], pidConstants[1], pidConstants[2]);

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
}

bool SFDrive::pidTurn (double degreesFromInit, int ticksLeft, int ticksRight, std::vector<double> pidConstants) {
	DriverStation::ReportError("Turning");
	if (pidInit == false) {
		SFDrive::PIDInit(pidConstants);
		pidInit = true;
	}
	SmartDashboard::PutNumber("Angle to Get", initialAngle + degreesFromInit);
	DriverStation::ReportError(std::to_string(initialAngle + degreesFromInit));

	//		frontMotorL->Set(x);
	//		frontMotorR->Set(-x);
	frontMotorL->Set(ticksLeft);
	frontMotorR->Set(ticksRight);

	if (abs(abs(navX->GetAngle()) - abs(initialAngle + degreesFromInit)) < 2) {
		DriverStation::ReportError("Done and returning true");
		//		ticksBackL = -backMotorL->GetEncPosition();
		//		ticksBackR = -backMotorR->GetEncPosition();
//		frontMotorL->Set(ticksBackL);
//		frontMotorR->Set(ticksBackR);
		//			SmartDashboard::PutNumber("Current Setpoint Left", ticksBackL);
		//			SmartDashboard::PutNumber("Current Setpoint Right", ticksBackR);

		return true;
	} else {
		//			resetMotors();

		if (degreesFromInit > 0) {
			if (abs(degreesFromInit) > 25) {
				ticksBackL -= tickRateTurnBig;
				ticksBackR -= tickRateTurnBig;
			} else {
				ticksBackL -= tickRateTurnSmall;
				ticksBackR -= tickRateTurnSmall;
			}
		} else {
			if (abs(degreesFromInit) > 25) {
				ticksBackL += tickRateTurnBig;
				ticksBackR += tickRateTurnBig;
			} else {
				ticksBackL += tickRateTurnSmall;
				ticksBackR += tickRateTurnSmall;
			}
		}

		return false;
	}
}

bool SFDrive::pidDrive (int ticksLeft, int ticksRight, std::vector<double> pidConstants) {

	//		frontMotorL->Set(x);
	//		frontMotorR->Set(-x);
	frontMotorL->Set((totalTicks > 0 ? ticksLeft : -ticksLeft));
	frontMotorR->Set((totalTicks > 0 ? -ticksRight : ticksRight));

	DriverStation::ReportError("Voltage: " + std::to_string(backMotorL->GetOutputVoltage()));
	// Checks if current tick position is within +-5% of expected total
	if (abs(abs(frontMotorL->GetEncPosition()) - abs(ticksLeft)) < abs(totalTicks * .03) &&
			abs(abs(frontMotorR->GetEncPosition()) - abs(ticksRight)) < abs(totalTicks * .03) &&
			abs(ticksLeft) >= abs(totalTicks) && abs(ticksRight) >= abs(totalTicks) &&
			abs(frontMotorL->GetOutputVoltage()) < 0.5 && abs(frontMotorR->GetOutputVoltage()) < 0.5) {
		DriverStation::ReportError("PID drive returning true");
		//			backMotorL->Set((totalTicks > 0 ? ticksLeft : -ticksLeft));
		//			backMotorR->Set((totalTicks > 0 ? -ticksRight : ticksRight));
		return true;
	} else {
		if (abs(ticksBackL) < abs(totalTicks)) {
			if (mediumDrive == true) {
				ticksBackL += tickRateForwardMedium;
			} else if (abs(totalTicks) < 60000) {
				ticksBackL += tickRateForwardSmall;
			} else {
				ticksBackL += tickRateForwardBig;
			}
		}
		if (abs(ticksBackR) < abs(totalTicks)) {
			if (mediumDrive == true) {
				ticksBackR += tickRateForwardMedium;
			} else if (abs(totalTicks) < 60000) {
				ticksBackR += tickRateForwardSmall;
			} else {
				ticksBackR += tickRateForwardBig;
			}
		}
		return false;
	}
}

double SFDrive::convertDistanceToTicks (double inches) {
	return inches / wheelDiameter / 3.1415 * 1024 * 4 * 60/14;
}
void SFDrive::resetMotors() {
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

// Some getters and setters
void SFDrive::setPIDInit(bool value) {
	pidInit = value;
}
void SFDrive::setArcadeInit(bool value) {
	arcadeInit = value;
}
int SFDrive::getSetpointLeft() {
	return ticksBackL;
}
int SFDrive::getSetpointRight() {
	return ticksBackR;
}

SFDrive::~SFDrive() {
	// TODO Auto-generated destructor stub
}

