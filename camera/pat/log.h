// Author: Ondrej Cierny
#ifndef __LOG
#define __LOG

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <sstream>
#include <ctime>
#include "packetdef.h"

template<typename... args> void log(zmq::socket_t& pat_health_port, std::ofstream& fileStream, args&&... msgs);
std::string timeStamp();

#endif
