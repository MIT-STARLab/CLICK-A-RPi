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
	std::string reader = "0";
	while(getline(fin, reader)){}  // get previous experiment number (or 0)
	int expNumPrev = stoi(reader); 
	int expNum = expNumPrev++;
	std::string expId = std::string::to_string(expNum);
	std::ofstream fout("/root/log/id_experiment.csv");
	fout << expId << std::endl;
	fout.close();
}
