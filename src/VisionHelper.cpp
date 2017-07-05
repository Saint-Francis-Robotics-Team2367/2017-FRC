/*
 * VisionHelper.cpp
 *
 *  Created on: Feb 11, 2017
 *      Author: Sameer Vijay
 */

#include <VisionHelper.h>
#include <cstring>

VisionHelper::VisionHelper() {
	description = "255.255.255.255";
	udpSocket = new UDPSocket(UDPPORT);
	//	udpSocket = new UDPSocket(description, UDPPORT);
	//	SetSocketBlockingEnabled(atoi(description.c_str()), false);
	hostIPReceived = false;
	//	hostIP = "10.23.67.182";
	//	tcpSocket = new TCPSocket(hostIP, TCPPORT);
}

bool VisionHelper::checkPendingPacket(int *last) {
	if (udpSocket->hasPendingPacket()) {
		receivePendingUDP();
		*last = std::stoi(udpReceiveString);
		return true;
	}
	return false;
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

bool VisionHelper::receivePendingUDP() {
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

