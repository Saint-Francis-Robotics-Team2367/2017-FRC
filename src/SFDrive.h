/*
 * SFDrive.h
 *
 *  Created on: Feb 6, 2017
 *      Author: Magneto
 */

#ifndef SRC_SFDRIVE_H_
#define SRC_SFDRIVE_H_

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

class SFDrive {

public:
	SFDrive(CANTalon *frontLeft, CANTalon *frontRight, CANTalon *backLeft, CANTalon *backRight);
	virtual ~SFDrive();

	CANTalon *frontMotorL;
	CANTalon *frontMotorR;
	CANTalon *backMotorL;
	CANTalon *backMotorR;

	AHRS *navX;
	float initialAngle;

	void PIDInit(double p, double i, double d);
	bool driveDistance (int ticksLeft, int ticksRight, double p, double i, double d);
	bool turnToAngle (double degreesFromInit, double ticksLeft, double ticksRight, double p, double i, double d);
	void joystickDrive(double forwardValue, double rotationValue);

private:

	frc::RobotDrive *drive;

	std::vector<double> pidDrive;
	std::vector<double> pidTurn;

	bool pidInit;
	int ticksBackL, ticksBackR, totalTicks;
	int totalCycles;

};

#endif /* SRC_SFDRIVE_H_ */
