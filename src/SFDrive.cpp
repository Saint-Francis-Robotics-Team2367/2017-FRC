/*
 * SFDrive.cpp
 *
 *  Created on: Feb 6, 2017
 *      Author: Magneto
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

#define tickRateForward 100
#define tickRateTurn 10
#define wheelDiameter 8

SFDrive::SFDrive(CANTalon *frontLeft, CANTalon *frontRight, CANTalon *backLeft, CANTalon *backRight) {
	frontMotorL = frontLeft;
	backMotorL = backLeft;
	frontMotorR = frontRight;
	backMotorR = backRight;

	drive = new RobotDrive(frontMotorL, backMotorL, frontMotorR, backMotorR);

	navX = new AHRS(SerialPort::kMXP);

	pidInit = false;
	ticksBackL = 0;
	ticksBackR = 0;
	totalTicks = 0;
	initialAngle = 0;
	totalCycles = 0;

	pidDrive = {0.24, 0, 0.18};
	pidTurn = {0.7, 0, 0};
}

void SFDrive::joystickDrive(double forwardValue, double rotationValue) {
	drive->ArcadeDrive(forwardValue, rotationValue);
}

void SFDrive::PIDInit(double p, double i, double d) {
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

bool SFDrive::turnToAngle (double degreesFromInit, double ticksLeft, double ticksRight, double p, double i, double d) {
	DriverStation::ReportError("Turning");
	// good turning PID's are 0.7, 0, 0
	if (pidInit == false) {
		SFDrive::PIDInit(p, i, d);
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

bool SFDrive::driveDistance (int ticksLeft, int ticksRight, double p, double i, double d) {
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

SFDrive::~SFDrive() {
	// TODO Auto-generated destructor stub
}

