#include "log.h"

// Get date and time for telemetry file names
//-----------------------------------------------------------------------------
std::string timeStamp()
//-----------------------------------------------------------------------------
{
	time_t     now = time(0);
	struct tm  tstruct;
	char       dateTime[80];
	tstruct = *localtime(&now);
	strftime(dateTime, sizeof(dateTime), "%Y-%m-%d-%H-%M-%S", &tstruct);
	return std::string(dateTime);
}
