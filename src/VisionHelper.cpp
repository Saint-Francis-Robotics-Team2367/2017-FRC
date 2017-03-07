/*
 * VisionHelper.cpp
 *
 *  Created on: Feb 11, 2017
 *      Author: Magneto
 */

#include <VisionHelper.h>
#include <cstring>

VisionHelper::VisionHelper() {
	udpSocket = new UDPSocket(UDPPORT);
	hostIPReceived = false;
//	hostIP = "10.23.67.182";
//	tcpSocket = new TCPSocket(hostIP, TCPPORT);
}

bool VisionHelper::receiveTCP() {
	int bytesReceived;

	while ((bytesReceived = (*tcpSocket).recv(tcpReceiveString, 999)) > 0) {
		return true;
	}
	return false;

	for (;;) {
		while ((bytesReceived = (*tcpSocket).recv(tcpReceiveString, 999)) > 0) { // Zero means end of transmission
//			cout << "Received: " << tcpReceiveString;
		}
//		sleep(1.0);
	}
	return true;
}

bool VisionHelper::receiveHostIP() {
	int bytesReceived = (*udpSocket).recvFrom(udpReceiveString, MAXSIZE, hostIP, udpPort);
	udpReceiveString[bytesReceived] = '\0';

	//	cout << "Received " << udpReceiveString << " from " << hostIP << ": " << udpPort << endl;

	if (bytesReceived == 0)
		return false;
	else {
//		tcpSocket = new TCPSocket(hostIP, TCPPORT);
		hostIPReceived = true;
		return true;
	}
}

VisionHelper::~VisionHelper() {
	// TODO Auto-generated destructor stub
}

