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
void log(std::ostream& stream, std::ofstream& fileStream, args&&... msgs)
//-----------------------------------------------------------------------------
{
	//log to terminal; eliminate this for flight
	using expand = int[];
	stream << "[" << std::fixed << std::setprecision(2) << (float)clock()/CLOCKS_PER_SEC << std::setprecision(5) << "] ";
	expand{0, (void(stream << std::forward<args>(msgs) << " "), 0)...};
	stream << std::endl;

	//log to text file
	fileStream << "[" << std::fixed << std::setprecision(2) << (float)clock()/CLOCKS_PER_SEC << std::setprecision(5) << "] ";
	expand{0, (void(fileStream << std::forward<args>(msgs) << " "), 0)...};
	fileStream << std::endl;
}


/*
//Temporary PAT health log (TODO: integrate into standard log)
template<typename... args>
void logToPatHealth(zmq::socket_t& pat_health_pub_port, args&&... msgs)
//-----------------------------------------------------------------------------
{	
	//log to pat health
	char healthBuffer[1024]; //TBR buffer size
	
	std::stringstream healthStream;
	healthStream.rdbuf()->pubsetbuf(healthBuffer, sizeof(healthBuffer));
	
	using expand = int[];
	healthStream << "[" << std::fixed << std::setprecision(2) << (float)clock()/CLOCKS_PER_SEC << std::setprecision(5) << "] ";
	expand{0, (void(healthStream << std::forward<args>(msgs) << " "), 0)...};
	healthStream << std::endl;	
	
	char* packet_pat_health = create_packet_pat_health(*healthBuffer);
	send_packet(pat_health_pub_port, packet_pat_health);
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
