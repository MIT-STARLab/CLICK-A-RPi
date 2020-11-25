// Author: Ondrej Cierny
#ifndef __LOG
#define __LOG

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <sstream>
#include "packetdef.h"

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
	
	using expand = int[];
	healthStream << "[" << std::fixed << std::setprecision(2) << (float)clock()/CLOCKS_PER_SEC << std::setprecision(5) << "] ";
	expand{0, (void(healthStream << std::forward<args>(msgs) << " "), 0)...};
	healthStream << std::endl;	
		
	send_packet_pat_health(pat_health_port, buffer);

	//log to text file
	fileStream << "[" << std::fixed << std::setprecision(2) << (float)clock()/CLOCKS_PER_SEC << std::setprecision(5) << "] ";
	expand{0, (void(fileStream << std::forward<args>(msgs) << " "), 0)...};
	fileStream << std::endl;	
	
	//log to terminal; eliminate this for flight
	std::cout << "[" << std::fixed << std::setprecision(2) << (float)clock()/CLOCKS_PER_SEC << std::setprecision(5) << "] ";
	expand{0, (void(std::cout << std::forward<args>(msgs) << " "), 0)...};
	std::cout << std::endl;	
}

/*
//Temporary PAT health log
template<typename... args>
void logToPatHealth(zmq::socket_t& pat_health_port, args&&... msgs)
//-----------------------------------------------------------------------------
{	
	char buffer[BUFFER_SIZE];	
	std::stringstream healthStream;
	healthStream.rdbuf()->pubsetbuf(buffer, sizeof(buffer));
	
	using expand = int[];
	healthStream << "[" << std::fixed << std::setprecision(2) << (float)clock()/CLOCKS_PER_SEC << std::setprecision(5) << "] ";
	expand{0, (void(healthStream << std::forward<args>(msgs) << " "), 0)...};
	healthStream << std::endl;	
	
	send_packet_pat_health(pat_health_port, buffer);
}
*/

// won't be needed for flight, Confirmable standard C-output version
/*
//-----------------------------------------------------------------------------
template<typename... args>
void logAndConfirm(args&&... msgs)
//-----------------------------------------------------------------------------
{
	using expand = int[];
	std::cout << "[" << std::fixed << std::setprecision(2) << (float)clock()/CLOCKS_PER_SEC << std::setprecision(5) << "] ";
	expand{0, (void(std::cout << std::forward<args>(msgs) << " "), 0)...};
	std::cin.ignore();
}
*/

#endif
