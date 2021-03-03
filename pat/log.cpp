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

bool is_number(const std::string& s)
{
    std::string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it)) ++it;
    return !s.empty() && it == s.end();
}

// Get folder number to save experiment data in
//-----------------------------------------------------------------------------
std::string getExperimentFolder(bool updateExpId)
//-----------------------------------------------------------------------------
{
	std::ifstream fin("/root/log/id_experiment.csv");
	std::string line;
	int expNum = 0;
	std::string expId;
	std::string directory_path;
	int expIds[10000]; //memory for reading experiment ids
	int linenum = 0;
	// get previous experiment number (or 0):
	while(getline(fin, line)){
		std::istringstream linestream(line);
		std::string item;
		getline(linestream, item); //convert to a string stream and then put in id.
		if(is_number(item)){
			std::stringstream ss(item);
			ss >> expIds[linenum];
		} else{
			if(linenum > 0){
				expIds[linenum] = expIds[linenum-1] + 1; //if not a number, use previous id + 1
			} else{
				expIds[linenum] = 0; //if first id is not a number, use zero to initialize
			}
		}	
		linenum++;
	}  
	expNum = expIds[linenum-1];
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
