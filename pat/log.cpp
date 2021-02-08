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
std::string getExperimentFolder(bool updateExpId)
//-----------------------------------------------------------------------------
{
	std::ifstream fin("/root/log/id_experiment.csv");
	std::string reader;
	int expNumPrev = 0;
	while(getline(fin, reader)){expNumPrev = stoi(reader);}  // get previous experiment number (or 0)
	int expNum = expNumPrev + 1;
	std::string expId = std::to_string(expNum);
	std::string directory_path = std::string("/root/log/pat/") + expId + std::string("/");
	if(updateExpId){
		//update experiement id list
		std::ofstream fout;
		fout.open("/root/log/id_experiment.csv", std::ios::app);
		fout << expId << std::endl;
		fout.close();

		//create new directory
		const int dir_err = mkdir(directory_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
		if(dir_err == -1){std::cout << "Experiment Directory Creation Error" << std::endl;}
	}
	return directory_path; 
}
