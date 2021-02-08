#include "log.h"

// Get date and time for telemetry file names
//-----------------------------------------------------------------------------
std::string timeStamp()
//-----------------------------------------------------------------------------
{
	time_t     now = time(0);
	struct tm  tstruct;
	char       dateTime[80];
	tstruct = *gmtime(&now); //localtime -> gmtime
	strftime(dateTime, sizeof(dateTime), "%Y-%m-%d-%H-%M-%S", &tstruct);
	return std::string(dateTime);
}

// Get folder number to save experiment data in
//-----------------------------------------------------------------------------
std::string getExperimentId()
//-----------------------------------------------------------------------------
{
	std::ifstream fin("/root/log/id_experiment.csv");
	std::string reader;
	int expNumPrev = 0;
	while(getline(fin, reader)){expNumPrev = stoi(reader);}  // get previous experiment number (or 0)
	int expNum = expNumPrev + 1;
	std::string expId = std::to_string(expNum);
	std::ofstream fout;
	fout.open("/root/log/id_experiment.csv", std::ios::app);
	fout << expId << std::endl;
	fout.close();
	return expId; 
}
