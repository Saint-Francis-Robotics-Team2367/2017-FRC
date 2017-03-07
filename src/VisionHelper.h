/*
 * VisionHelper.h
 *
 *  Created on: Feb 11, 2017
 *      Author: Magneto
 */

#ifndef SRC_VISIONHELPER_H_
#define SRC_VISIONHELPER_H_

#include "PracticalSocket.h"
#include <DriverStation.h>
#define MAXSIZE 999
#define TCPPORT 2367
#define UDPPORT 7330

class VisionHelper {
public:
	UDPSocket *udpSocket;
	unsigned short udpPort = UDPPORT;
	string hostIP = "255.255.255.255";
	char udpReceiveString[MAXSIZE + 1];

	TCPSocket *tcpSocket;
	char tcpReceiveString[1024];

	bool hostIPReceived;

	VisionHelper();
	bool receiveHostIP();
	bool receiveTCP();
	virtual ~VisionHelper();
};

#endif /* SRC_VISIONHELPER_H_ */
