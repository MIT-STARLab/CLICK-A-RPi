// Author: Ondrej Cierny
#ifndef __LOG
#define __LOG

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <sstream>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdlib.h>
#include "packetdef.h"

#define MAX_LOG_NUMBER 100 //maximum number of logs to keep in memory (delete older logs on startup)

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
	healthStream << "[" << std::fixed << std::setprecision(2) << ((float)clock())/CLOCKS_PER_SEC << std::setprecision(5) << "] ";
	(void) expander{0, (void(healthStream << std::forward<args>(msgs) << " "), 0)...};
	healthStream << std::endl;	
		
	send_packet_pat_health(pat_health_port, buffer);

	//log to text file
	fileStream << "[" << std::fixed << std::setprecision(2) << ((float)clock())/CLOCKS_PER_SEC << std::setprecision(5) << "] ";
	(void) expander{0, (void(fileStream << std::forward<args>(msgs) << " "), 0)...};
	fileStream << std::endl;	
	
	//log to terminal; eliminate this for flight
	/*
	std::cout << "[" << std::fixed << std::setprecision(2) << (float)clock()/CLOCKS_PER_SEC << std::setprecision(5) << "] ";
	(void) expander{0, (void(std::cout << std::forward<args>(msgs) << " "), 0)...};
	std::cout << std::endl;	
	*/
}

// Get date and time for telemetry file names
std::string timeStamp();

// Check if read experiment id string is a number
bool is_number(const std::string& s);

//Get experiment id number
int getExperimentId();

// Get folder path to save experiment data in
std::string getExperimentFolder(bool updateExpId = false);

//Delete old logs
void manageLogs();

//Write line to CSV data file
void writeToDataFile(std::ofstream& fileStream, double time, double bcnX, double bcnY, double bcnExp, double calX, double calY, double calSetX, double calSetY, double calExp);

//Write header line to CSV data file
void writeHeaderToDataFile(std::ofstream& fileStream);


#endif
