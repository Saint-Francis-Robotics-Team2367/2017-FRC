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
	int totalTicks;

	double convertDistanceToTicks (double inches);
	bool driveDistance(double ticks);
	bool turnToAngle(double angle);
	void joystickDrive(double forwardValue, double rotationValue);

	void resetMotors();

	void setPIDInit(bool value);
	void setArcadeInit(bool value);
	int getSetpointLeft();
	int getSetpointRight();

private:
	frc::RobotDrive *drive;

	std::vector<double> pidDriveVals;
	std::vector<double> pidTurnVals;

	bool arcadeInit, pidInit;
	int ticksBackL, ticksBackR;
	int totalCycles;

	void ArcadeInit();
	void PIDInit(std::vector<double> pidConstants);
	bool pidDrive (int ticksLeft, int ticksRight, std::vector<double> pidConstants);
	bool pidTurn (double degreesFromInit, int ticksLeft, int ticksRight, std::vector<double> pidConstants);
};

#endif /* SRC_SFDRIVE_H_ */
