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
	int expNum = 0;
	std::string expId;
	std::string directory_path;

	while(getline(fin, reader)){expNum = stoi(reader);}  // get previous experiment number (or 0)
	if(updateExpId){
		//update from previous experiment id
		expNum += 1;
		expId = std::to_string(expNum);
		directory_path = std::string("/root/log/pat/") + expId;

		//update experiment id list
		std::ofstream fout;
		fout.open("/root/log/id_experiment.csv", std::ios::app);
		fout << expId << std::endl;
		fout.close();

		//create new directory
		const int dir_err = mkdir(directory_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
		if(dir_err == -1){std::cout << "Experiment Directory Creation Error" << std::endl;}
	} else{
		//output current experiment id
		expId = std::to_string(expNum);
		directory_path = std::string("/root/log/pat/") + expId;
	}
	return directory_path + std::string("/"); 
}
