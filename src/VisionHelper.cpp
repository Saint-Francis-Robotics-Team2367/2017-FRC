/*
 * VisionHelper.cpp
 *
 *  Created on: Feb 11, 2017
 *      Author: Magneto
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

#include <fcntl.h>

/** Returns true on success, or false if there was an error */
bool VisionHelper::SetSocketBlockingEnabled(int fd, bool blocking)
{
   if (fd < 0) return false;

#ifdef WIN32
   unsigned long mode = blocking ? 0 : 1;
   return (ioctlsocket(fd, FIONBIO, &mode) == 0) ? true : false;
#else
   int flags = fcntl(fd, F_GETFL, 0);
   if (flags < 0) return false;
   flags = blocking ? (flags&~O_NONBLOCK) : (flags|O_NONBLOCK);
   return (fcntl(fd, F_SETFL, flags) == 0) ? true : false;
#endif
}

VisionHelper::~VisionHelper() {
	// TODO Auto-generated destructor stub
}

