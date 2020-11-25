// BlueFox camera wrapper class
// Authors: Ondrej Cierny, Peter Grenfell
#include "camera.h"
#include <cstdlib>

//-----------------------------------------------------------------------------
Camera::Camera(std::ofstream &fileStreamIn, zmq::socket_t &pat_health_port_in, std::string serialNumber): 
fileStream(fileStreamIn), pat_health_port(pat_health_port_in)
//-----------------------------------------------------------------------------
{
	if(initialize(serialNumber))
	{
		log(pat_health_port, fileStream, "In camera.cpp Camera::Camera - Camera Connection Initialized");
	}
}

//-----------------------------------------------------------------------------
Camera::~Camera()
//-----------------------------------------------------------------------------
{
	delete config;
	delete fi;
}

//-----------------------------------------------------------------------------
bool Camera::initialize(std::string serialNumber)
//-----------------------------------------------------------------------------
{
	device = NULL;
	queuedCount = 0;
	requestQueueSize = 2;

	// Check if camera is connected
	for(unsigned int i = 0; i < manager.deviceCount(); i++)
	{
		if(manager[i]->serial.read().find(serialNumber) == 0)
		{
			try
			{
				manager[i]->open();
				device = manager[i];
			}
			catch(...) {}
		}
	}

	if(device == NULL)
	{
		error = ERROR_NULL_DEVICE;
		return false;
	}
	else
	{
		// Basic configuration
		fi = new FunctionInterface(device);
		config = new CameraSettingsBlueFOX(device);
		config->triggerMode.write(ctmOnDemand);
		config->pixelFormat.write(ibpfMono10);
		return true;
	}
}

//-----------------------------------------------------------------------------
bool Camera::requestFrame()
//-----------------------------------------------------------------------------
{
	int result = fi->imageRequestSingle();
	if(result != DMR_NO_ERROR)
	{
		error = ImpactAcquireException::getErrorCodeAsString(result);
		log(pat_health_port, fileStream, "Error requesting", config->aoiWidth.read(), "x", config->aoiHeight.read(), "frame!", error);
		return false;
	}
	// log(pat_health_port, "Requested", config->aoiWidth.read(), "x", config->aoiHeight.read(),
	// 	"at", config->aoiStartX.read(), ",", config->aoiStartY.read());
	queuedCount++;
	return true;
}

//-----------------------------------------------------------------------------
bool Camera::waitForFrame()
//-----------------------------------------------------------------------------
{
	frameId = fi->imageRequestWaitFor(CAMERA_MAX_WAIT);
	if(fi->isRequestNrValid(frameId))
	{
		const Request* request = fi->getRequest(frameId);
		if(request->isOK()) return true;
		else
		{
			error = request->requestResult.readS();
			unlockRequest();
		}
	}
	else error = ImpactAcquireException::getErrorCodeAsString(frameId);
	return false;
}

//-----------------------------------------------------------------------------
bool Camera::fillRequestQueue()
//-----------------------------------------------------------------------------
{
	for(int i = 0; i < requestQueueSize; i++)
	{
		if(!requestFrame()) return false;
	}
	return true;
}

//-----------------------------------------------------------------------------
void Camera::clearRequestQueue()
//-----------------------------------------------------------------------------
{
	fi->imageRequestReset(0, 0);
}

//-----------------------------------------------------------------------------
const Request* Camera::getRequest()
//-----------------------------------------------------------------------------
{
	return fi->getRequest(frameId);
}

//-----------------------------------------------------------------------------
void Camera::ignoreNextFrames(int n)
//-----------------------------------------------------------------------------
{
	// if(n > 0) log(pat_health_port, "Ignoring", n, "pre-queued frames");
	for(int i = 0; i < n; i++)
	{
		waitForFrame();
		unlockRequest();
	}
}

//-----------------------------------------------------------------------------
void Camera::unlockRequest()
//-----------------------------------------------------------------------------
{
	if(fi->isRequestNrValid(frameId))
	{
		fi->imageRequestUnlock(frameId);
		queuedCount--;
	}
}

//-----------------------------------------------------------------------------
void Camera::setFullWindow()
//-----------------------------------------------------------------------------
{
	config->aoiStartX.write(0);
	config->aoiStartY.write(0);
	config->aoiWidth.write(CAMERA_WIDTH);
	config->aoiHeight.write(CAMERA_HEIGHT);
}

//-----------------------------------------------------------------------------
void Camera::setWindow(AOI& window)
//-----------------------------------------------------------------------------
{
	if(config->aoiWidth.read() != window.w || config->aoiHeight.read() != window.h ||
	   config->aoiStartX.read() != window.x || config->aoiStartY.read() != window.y)
	{
		config->aoiStartX.write(window.x);
		config->aoiStartY.write(window.y);
		config->aoiWidth.write(window.w);
		config->aoiHeight.write(window.h);
	}
}

// Calculate and set a centered window
// This is an approximation (not quantized to multiples of 16/8)
//-----------------------------------------------------------------------------
void Camera::setCenteredWindow(int x, int y, int w)
//-----------------------------------------------------------------------------
{
	int h = w;
	// Calculate correct offsets and clamp to canvas
	if(x > w/2)
	{
		x -= w/2;
		if(x + w > CAMERA_WIDTH) w = CAMERA_WIDTH - x;
	}
	else
	{
		w = x + w/2;
		x = 0;
	}
	if(y > h/2)
	{
		y -= h/2;
		if(y + h > CAMERA_HEIGHT) h = CAMERA_HEIGHT - y;
	}
	else
	{
		h = y + h/2;
		y = 0;
	}

	// Write parameters
	config->aoiStartX.write(x);
	config->aoiStartY.write(y);
	config->aoiWidth.write(w);
	config->aoiHeight.write(h);
}
