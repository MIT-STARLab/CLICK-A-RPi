// BlueFox camera wrapper class
// Authors: Ondrej Cierny, Peter Grenfell
#ifndef __CAMERA
#define __CAMERA
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include "log.h"

#define CAMERA_WIDTH 2592
#define CAMERA_HEIGHT 1944
#define CAMERA_MAX_WAIT 5000	// Max frame timeout in ms
#define SERIAL_NUMBER "297"
#define ERROR_NULL_DEVICE "NULL device"

// An area of interest on camera
class AOI
{
public:
	int x, y, w, h;
	AOI() : x(0), y(0), w(CAMERA_WIDTH), h(CAMERA_HEIGHT) {}
};

// Base camera class
class Camera
{
	int frameId;
	Device *device;
	DeviceManager manager;
	FunctionInterface *fi;
	std::ofstream &fileStream;
	zmq::socket_t &pat_health_port;
public:
	std::string error;
	int requestQueueSize, queuedCount;
	CameraSettingsBlueFOX *config;
	Camera(std::ofstream &fileStreamIn, zmq::socket_t &pat_health_port_in, std::string serialNumber = SERIAL_NUMBER);
	~Camera();
	bool initialize(std::string serialNumber = SERIAL_NUMBER);
	bool requestFrame();
	bool waitForFrame();
	bool fillRequestQueue();
	void clearRequestQueue();
	void ignoreNextFrames(int n = 1);
	void unlockRequest();
	void setFullWindow();
	void setWindow(AOI& window);
	void setCenteredWindow(int x, int y, int w);
	const Request* getRequest();
};

#endif
