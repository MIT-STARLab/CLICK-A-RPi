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

// Variable argument logging function with clock timestamp
//-----------------------------------------------------------------------------
template<typename... args>
void log(zmq::socket_t& pat_health_port, std::ofstream& fileStream, args&&... msgs)
//-----------------------------------------------------------------------------
{
	//log to PAT health for housekeeping telemetry
	char buffer[BUFFER_SIZE];	
	std::stringstream healthStream;
	healthStream.rdbuf()->pubsetbuf(buffer, sizeof(buffer));
	
	using expander = int[];
	healthStream << "[" << std::fixed << std::setprecision(2) << (float)clock()/CLOCKS_PER_SEC << std::setprecision(5) << "] ";
	(void) expander{0, (void(healthStream << std::forward<args>(msgs) << " "), 0)...};
	healthStream << std::endl;	
		
	send_packet_pat_health(pat_health_port, buffer);

	//log to text file
	fileStream << "[" << std::fixed << std::setprecision(2) << (float)clock()/CLOCKS_PER_SEC << std::setprecision(5) << "] ";
	(void) expander{0, (void(fileStream << std::forward<args>(msgs) << " "), 0)...};
	fileStream << std::endl;	
	
	//log to terminal; eliminate this for flight
	std::cout << "[" << std::fixed << std::setprecision(2) << (float)clock()/CLOCKS_PER_SEC << std::setprecision(5) << "] ";
	(void) expander{0, (void(std::cout << std::forward<args>(msgs) << " "), 0)...};
	std::cout << std::endl;	
}
