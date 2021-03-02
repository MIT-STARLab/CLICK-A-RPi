#include <unistd.h>
#include <atomic>
#include <csignal>
//#include <chrono>
//#include <thread>
#include <sstream>
#include <stdio.h>

#include "log.h"
#include "fsm.h"
#include "camera.h"
#include "processing.h"
#include "tracking.h"
#include "calibration.h"

#define CALIB_CH 0x20 //calib laser fpga channel (Notated_memory_map on Google Drive)
#define CALIB_ON 0x55 //calib laser ON code (Notated_memory_map on Google Drive)
#define CALIB_OFF 0x0F //calib laser OFF code (Notated_memory_map on Google Drive)
#define HEATER_CH 0x23 //heater fpga channel (Notated_memory_map on Google Drive)
#define HEATER_ON 0x55 //heater ON code (Notated_memory_map on Google Drive)
#define HEATER_OFF 0x0F //heater OFF code (Notated_memory_map on Google Drive)
#define TEMPERATURE_CH 202 //Camera Temperature
#define CENTROID2ANGLE_SLOPE_X -0.0000986547085f //user input from calibration
#define CENTROID2ANGLE_BIAS_X 0.127856502f //user input from calibration
#define CENTROID2ANGLE_SLOPE_Y -0.0000986547085f //user input from calibration
#define CENTROID2ANGLE_BIAS_Y 0.095892377f //user input from calibration
#define PERIOD_BEACON_LOSS 3.0f //seconds, time to wait after beacon loss before switching back to acquisition
#define PERIOD_CALIB_LOSS 3.0f //seconds, time to wait after beacon loss before switching to open loop
#define PERIOD_HEARTBEAT_TLM 0.5f //seconds, time to wait in between heartbeat telemetry messages
#define PERIOD_CSV_WRITE 0.1f //seconds, time to wait in between writing csv telemetry data
#define PERIOD_TX_ADCS 1.0f //seconds, time to wait in between bus adcs feedback messages
#define PERIOD_CALCULATE_TX_OFFSETS 1000.0f //seconds, time to wait in-between updating tx offsets due to temperature fluctuations
#define PERIOD_DITHER_TX_OFFSETS 10.0f //seconds, time to wait in-between dithering tx offsets (if dithering is on)
#define LASER_RISE_TIME 10 //milliseconds, time to wait after switching the cal laser on/off (min rise time = 3 ms)
#define TX_OFFSET_X_DEFAULT -15 //pixels, from GSE calibration [old: 20] [new = 2*caliboffset + 20]
#define TX_OFFSET_Y_DEFAULT 194 //pixels, from GSE calibration [old: -50] [new = 2*caliboffset - 50]
#define CALIB_EXPOSURE_SELF_TEST 25 //microseconds, default if autoexposure fails for self tests
#define CALIB_OFFSET_TOLERANCE 100 //maximum acceptable calibration offset for self tests
#define CALIB_SENSITIVITY_RATIO_TOL 0.1 //maximum acceptable deviation from 1/sqrt(2) for sensitivity ratio = s00/s11
#define BCN_X_REL_GUESS 0 //estimate of beacon x position on acquisition rel to center
#define BCN_Y_REL_GUESS 0 //estimate of beacon y position on acquisition rel to center
#define TX_OFFSET_SLOPE_X 0.3462f //TBD, pxls/C - linear coeff of tx offset as a function of temperature
#define TX_OFFSET_BIAS_X -22.464f //TBD, pxls - bias coeff of tx offset as a function of temperature
#define TX_OFFSET_QUADRATIC_Y 0.0085f //TBD, pxls/C^2 - quadratic coeff of tx offset as a function of temperature
#define TX_OFFSET_SLOPE_Y -0.3471f //TBD, pxls/C - linear coeff of tx offset as a function of temperature
#define TX_OFFSET_BIAS_Y 196.46f //TBD, pxls - bias coeff of tx offset as a function of temperature
#define TX_OFFSET_DITHER_X_RADIUS 4.0f //pxls
#define TX_OFFSET_DITHER_Y_RADIUS 1.0f //pxls
#define DITHER_COUNT_PERIOD 10 //1 full period after 10 ditherings

using namespace std;
using namespace std::chrono;

class CSVdata
{
public:
	double bcnX, bcnY, bcnExp, calX, calY, calSetX, calSetY, calExp;
	CSVdata(double bcnXin, double bcnYin, double bcnExpIn,
    double calXin, double calYin, double calSetXin, double calSetYin, double calExpIn) :
    bcnX(bcnXin), bcnY(bcnYin), bcnExp(bcnExpIn),
    calX(calXin), calY(calYin), calSetX(calSetXin), calSetY(calSetYin), calExp(calExpIn) {}
};

//Turn on Calibration Laser
bool laserOn(zmq::socket_t& pat_health_port, std::ofstream& fileStream, zmq::socket_t& fpga_map_request_port, zmq::socket_t& fpga_map_answer_port, std::vector<zmq::pollitem_t>& poll_fpga_answer,  uint8_t request_number = 0){
	bool laser_is_on;
	if(check_fpga_map_value(fpga_map_answer_port, poll_fpga_answer, fpga_map_request_port, (uint16_t) HEATER_CH, (uint8_t) HEATER_ON, request_number)){
		// Heater is ON -> Send laser ON command to FPGA:
		send_packet_fpga_map_request(fpga_map_request_port, (bool) WRITE, request_number, (uint16_t) CALIB_CH, (uint8_t) CALIB_ON);
		// Check that message was received and FPGA was written to:
		laser_is_on = check_fpga_map_write_request(fpga_map_answer_port, poll_fpga_answer, (uint16_t) CALIB_CH, request_number);
		std::this_thread::sleep_for(std::chrono::milliseconds(LASER_RISE_TIME));
		return laser_is_on;
	} else{
		// Heater is OFF (or the read request failed)
		log(pat_health_port, fileStream, "In main.cpp - laserOn: Heater ON check did not pass. Commanding Heater ON...");
		// Send Heater ON command to FPGA:
		send_packet_fpga_map_request(fpga_map_request_port, (bool) WRITE, request_number, (uint16_t) HEATER_CH, (uint8_t) HEATER_ON);
		// Check that message was received and FPGA was written to:
		if(check_fpga_map_write_request(fpga_map_answer_port, poll_fpga_answer, (uint16_t) HEATER_CH, request_number)){
			// Heater is ON -> Send laser ON command to FPGA:
			send_packet_fpga_map_request(fpga_map_request_port, (bool) WRITE, request_number, (uint16_t) CALIB_CH, (uint8_t) CALIB_ON);
			// Check that message was received and FPGA was written to:
			laser_is_on = check_fpga_map_write_request(fpga_map_answer_port, poll_fpga_answer, (uint16_t) CALIB_CH, request_number);
			std::this_thread::sleep_for(std::chrono::milliseconds(LASER_RISE_TIME));
			return laser_is_on;
		} else{
			log(pat_health_port, fileStream, "In main.cpp - laserOn: Heater ON write failed!");
			return laser_is_on = false;
		}
	}
}

//Turn Off Calibration Laser
bool laserOff(zmq::socket_t& fpga_map_request_port, zmq::socket_t& fpga_map_answer_port, std::vector<zmq::pollitem_t>& poll_fpga_answer, uint8_t request_number = 0){
	send_packet_fpga_map_request(fpga_map_request_port, (bool) WRITE, request_number, (uint16_t) CALIB_CH, (uint8_t) CALIB_OFF);
	// Check that message was received and FPGA was written to:
	bool return_val = check_fpga_map_write_request(fpga_map_answer_port, poll_fpga_answer, (uint16_t) CALIB_CH, request_number);
	std::this_thread::sleep_for(std::chrono::milliseconds(LASER_RISE_TIME));
	return return_val;
}

//Convert Beacon Centroid to Error Angles for the Bus
struct error_angles{
	float angle_x_radians;
	float angle_y_radians;
};
error_angles centroid2angles(double centroid_x, double centroid_y){
	error_angles angles = error_angles();
	angles.angle_x_radians = (float) TX_OFFSET_SLOPE_X*centroid_x + CENTROID2ANGLE_BIAS_X;
	angles.angle_y_radians = (float) CENTROID2ANGLE_SLOPE_Y*centroid_y + CENTROID2ANGLE_BIAS_Y;
	return angles;
}

//for loading tx offset parameters from external file
enum offsetParamIndex { 
    IDX_PERIOD_CALCULATE_TX_OFFSETS, 
    IDX_PERIOD_DITHER_TX_OFFSETS,
    IDX_TX_OFFSET_X_DEFAULT,
    IDX_TX_OFFSET_Y_DEFAULT,
    IDX_TX_OFFSET_SLOPE_X,
    IDX_TX_OFFSET_BIAS_X,
    IDX_TX_OFFSET_QUADRATIC_Y,
    IDX_TX_OFFSET_SLOPE_Y,
    IDX_TX_OFFSET_BIAS_Y,
    IDX_TX_OFFSET_DITHER_X_RADIUS,
    IDX_TX_OFFSET_DITHER_Y_RADIUS,
    IDX_DITHER_COUNT_PERIOD,
    NUM_TX_OFFSET_PARAMS
};
struct offsetParamStruct {
    string name;
    float parameter;
};
bool getOffsetParams(zmq::socket_t& pat_health_port, std::ofstream& fileStream, offsetParamStruct (&offsetParamsIn)[NUM_TX_OFFSET_PARAMS]){
	offsetParamStruct offsetParam; //temp offsetParamStruct for use in the while loop
    ifstream inFile("/root/lib/offsetParams.csv"); //our file
    string line;
    int linenum = 0;
	log(pat_health_port, fileStream, "In main.cpp - getOffsetParams: Retrieving Tx Offset Parameters from /root/lib/offsetParams.csv");
    if(inFile.is_open()){
		while (getline (inFile, line))
		{
			istringstream linestream(line);
			string item;
			//use this to get up to the first comma
			getline(linestream, item, ',');
			offsetParam.name = item;
			//convert to a string stream and then put in id.
			getline(linestream, item, ',');
			stringstream ss(item);
			ss >> offsetParam.parameter;
			//report read data
			log(pat_health_port, fileStream, "In main.cpp - getOffsetParams: ", offsetParam.name, ": ", offsetParam.parameter);
			//add the new data to the list
			offsetParamsIn[linenum] = offsetParam;
			linenum++;
		}
		return true;
	} else{
		log(pat_health_port, fileStream, "In main.cpp - getOffsetParams: /root/lib/offsetParams.csv did not open or doesn't exist.");
		return false;
	}
}

//Get payload temperature and compute Tx offsets
struct tx_offsets{
	float x;
	float y;
};
void calculateTxOffsets(zmq::socket_t& pat_health_port, std::ofstream& fileStream, zmq::socket_t& fpga_map_request_port, zmq::socket_t& fpga_map_answer_port, std::vector<zmq::pollitem_t>& poll_fpga_answer, tx_offsets& offsets){
	fpga_answer_temperature_struct temperature_packet = fpga_answer_temperature_struct();
	if(get_temperature(fpga_map_answer_port, poll_fpga_answer, fpga_map_request_port, temperature_packet, (uint16_t) TEMPERATURE_CH, 0)){
		offsets.x = TX_OFFSET_SLOPE_X*temperature_packet.temperature + TX_OFFSET_BIAS_X;
		offsets.y = TX_OFFSET_QUADRATIC_Y*temperature_packet.temperature*temperature_packet.temperature + TX_OFFSET_SLOPE_Y*temperature_packet.temperature + TX_OFFSET_BIAS_Y;
		log(pat_health_port, fileStream, "In main.cpp - calculateTxOffsets: Temperature Reading = ", temperature_packet.temperature, ", offsets.x = ", offsets.x, ", offsets.y = ", offsets.y);
	} else{
		log(pat_health_port, fileStream, "In main.cpp - calculateTxOffsets: FPGA temperature read failed. Using offsets.x = ", offsets.x, ", offsets.y = ", offsets.y);
	}
}

void ditherOffsets(zmq::socket_t& pat_health_port, std::ofstream& fileStream, tx_offsets& offsets, int count, float offset_x_init, float offset_y_init){
	count = count%DITHER_COUNT_PERIOD;
	float t = (float) count / ((float) DITHER_COUNT_PERIOD);
	offsets.x = t * TX_OFFSET_DITHER_X_RADIUS * cos(2 * M_PI * t) + offset_x_init;
	offsets.y = t * TX_OFFSET_DITHER_Y_RADIUS * sin(2 * M_PI * t) + offset_y_init;
	log(pat_health_port, fileStream, "In main.cpp - ditherOffsets: Updating to offsets.x = ", offsets.x, ", offsets.y = ", offsets.y);
}

atomic<bool> stop(false);

//-----------------------------------------------------------------------------
int main() //int argc, char** argv
//-----------------------------------------------------------------------------
{	
	// https://ogbe.net/blog/zmq_helloworld.html
	// define ports for PUB/SUB (this process binds)
	std::string PAT_STATUS_PORT = "tcp://127.0.0.1:5564"; //PUB to Status
	std::string PAT_HEALTH_PORT = "tcp://127.0.0.1:5559"; //PUB to Housekeeping
    std::string PAT_CONTROL_PORT = "tcp://127.0.0.1:5560"; //SUB to Command Handler
    std::string FPGA_MAP_REQUEST_PORT = "tcp://localhost:5558"; //PUB to FPGA Driver
    std::string FPGA_MAP_ANSWER_PORT = "tcp://localhost:5557"; //SUB to FPGA Driver
    std::string TX_PACKETS_PORT = "tcp://localhost:5561"; //PUB to Packetizer & Bus Interface
    //std::string RX_PAT_PACKETS_PORT = "tcp://localhost:5562";  //SUB to Packetizer & Bus Interface 

    // initialize the zmq context with 1 IO threads 
    zmq::context_t context{1}; 
    
    // Create the PUB/SUB Sockets: 
	//int linger = 0; // Configure sockets to not wait at close time
	//int rc;
	
	// create the PAT_STATUS_PORT PUB socket
	zmq::socket_t pat_status_port(context, ZMQ_PUB); 
	//rc = zmq_setsockopt(pat_status_port, ZMQ_LINGER, &linger, sizeof(linger));
    pat_status_port.bind(PAT_STATUS_PORT); // connect to the transport bind(PAT_HEALTH_PORT)
	//std::cout << "rc (pat_status_port): " << rc << std::endl;

	// create the PAT_HEALTH_PORT PUB socket
	zmq::socket_t pat_health_port(context, ZMQ_PUB); 
	//rc = zmq_setsockopt(pat_health_port, ZMQ_LINGER, &linger, sizeof(linger));
    pat_health_port.bind(PAT_HEALTH_PORT); // connect to the transport bind(PAT_HEALTH_PORT)
	//std::cout << "rc (pat_health_port): " << rc << std::endl;
    
    // create the PAT_CONTROL_PORT SUB socket
    zmq::socket_t pat_control_port(context, ZMQ_SUB); 
	//rc = zmq_setsockopt(pat_control_port, ZMQ_LINGER, &linger, sizeof(linger));
    pat_control_port.bind(PAT_CONTROL_PORT); // connect to the transport
    pat_control_port.set(zmq::sockopt::subscribe, ""); // set the socket options such that we receive all messages. we can set filters here. this "filter" ("" and 0) subscribes to all messages.	
	//std::cout << "rc (pat_control_port): " << rc << std::endl;

    // create the FPGA_MAP_REQUEST_PORT PUB socket
    zmq::socket_t fpga_map_request_port(context, ZMQ_PUB); 
	//rc = zmq_setsockopt(fpga_map_request_port, ZMQ_LINGER, &linger, sizeof(linger));
    fpga_map_request_port.connect(FPGA_MAP_REQUEST_PORT); // connect to the transport
	//std::cout << "rc (fpga_map_request_port): " << rc << std::endl;

    // create the FPGA_MAP_ANSWER_PORT SUB socket
    zmq::socket_t fpga_map_answer_port(context, ZMQ_SUB); // create the FPGA_MAP_ANSWER_PORT SUB socket
	//rc = zmq_setsockopt(fpga_map_answer_port, ZMQ_LINGER, &linger, sizeof(linger));
    fpga_map_answer_port.connect(FPGA_MAP_ANSWER_PORT); // connect to the transport
	uint32_t pat_pid = getpid();
	char subscription[sizeof(pat_pid)];
	memcpy(subscription, &pat_pid, sizeof(subscription));	
	//std::cout << "PAT PID: " << subscription << std::endl;
    fpga_map_answer_port.set(zmq::sockopt::subscribe, subscription); // set the socket options such that we receive all messages. we can set filters here. this "filter" ("" and 0) subscribes to all messages.	
	//std::cout << "rc (fpga_map_answer_port): " << rc << std::endl;

    // create the TX_PACKETS_PORT PUB socket
    zmq::socket_t tx_packets_port(context, ZMQ_PUB); 
	//rc = zmq_setsockopt(tx_packets_port, ZMQ_LINGER, &linger, sizeof(linger));
    tx_packets_port.connect(TX_PACKETS_PORT); // connect to the transport
	//std::cout << "rc (tx_packets_port): " << rc << std::endl;

    /*
    // create the RX_PAT_PACKETS_PORT SUB socket
    zmq::socket_t rx_pat_packets_port(context, ZMQ_SUB); 
    rx_pat_packets_port.connect(RX_PAT_PACKETS_PORT); // connect to the transport
    rx_pat_packets_port.set(zmq::sockopt::subscribe, ""); // set the socket options such that we receive all messages. we can set filters here. this "filter" ("" and 0) subscribes to all messages.	
	*/
	
	// to use zmq_poll correctly, we construct this vector of pollitems (based on: https://ogbe.net/blog/zmq_helloworld.html)
	std::vector<zmq::pollitem_t> poll_pat_control = {{pat_control_port, 0, ZMQ_POLLIN, 0}};
	std::vector<zmq::pollitem_t> poll_fpga_answer = {{fpga_map_answer_port, 0, ZMQ_POLLIN, 0}};
	
	//Allow sockets some time (otherwise you get dropped packets)
	std::this_thread::sleep_for(std::chrono::seconds(1));
		
	//telemetry file names
	std::string pathName = getExperimentFolder(true); //save path, get experiment id, update exp id csv file, make experiment folder
	std::string textFileName = pathName + timeStamp() + string("_pat_logs.txt"); //used for text telemetry
	std::string dataFileName = pathName + timeStamp() + string("_pat_data.csv"); //used by csv data file generation

	//Generate text telemetry file, pg
	ofstream textFileOut; //stream for text telemetry
	textFileOut.open(textFileName, ios::app); //create text file and open for writing
	log(pat_health_port, textFileOut, "In main.cpp - Started PAT with PID: ", pat_pid);

	// Synchronization
	enum Phase { CALIBRATION, ACQUISITION, CL_INIT, CL_BEACON, CL_CALIB, OPEN_LOOP, STATIC_POINT };
	const char *phaseNames[8] = {"CALIBRATION","ACQUISITION","CL_INIT","CL_BEACON","CL_CALIB","OPEN_LOOP","STATIC_POINT"};
	
	// Initialize execution variables
	Phase phase = CALIBRATION;
	Group beacon, calib;
	AOI beaconWindow, calibWindow, getImageWindow;
	//set default get image window
	getImageWindow.w = CALIB_BIG_WINDOW;
	getImageWindow.h = CALIB_BIG_WINDOW;
	getImageWindow.x = CAMERA_WIDTH/2 - CALIB_BIG_WINDOW/2;
	getImageWindow.y = CAMERA_HEIGHT/2 - CALIB_BIG_WINDOW/2;
	int cmd_get_image_w = getImageWindow.w, cmd_get_image_h = getImageWindow.h;
	int cmd_get_image_center_x_rel = 0, cmd_get_image_center_y_rel = 0;
	int beaconExposure = 0, spotIndex = 0, calibExposure = 0;
	int beaconGain = 0, calibGain = 0;
	bool haveBeaconKnowledge = false, haveCalibKnowledge = false;
	double propertyDifference = 0;
	bool openLoop = false, staticPoint = false, sendBusFeedback = false, bcnAlignment = false; 
	uint16_t command; 
	int command_exposure;  
	int cl_beacon_num_groups, cl_calib_num_groups;
	int num_calibration_attempts = 0, num_acquisition_attempts = 0; 
	bool static_pointing_initialized = false;
	int period_hb_tlm_ms = (int) 1000*PERIOD_HEARTBEAT_TLM;
	bool OPERATIONAL = true, STANDBY = true; 
	uint8_t camera_test_result, fpga_test_result, laser_test_result, fsm_test_result, calibration_test_result;
	int command_offset_x, command_offset_y; 
	int main_entry_flag;
	int beaconWindowSize = TRACK_ACQUISITION_WINDOW;
	beaconWindow.w = beaconWindowSize;
	beaconWindow.h = beaconWindow.w;
	int beacon_x_rel = BCN_X_REL_GUESS, beacon_y_rel = BCN_Y_REL_GUESS;
	beacon.x = CAMERA_WIDTH/2 + beacon_x_rel; beacon.y = CAMERA_HEIGHT/2 + beacon_y_rel;
	int maxBcnExposure = TRACK_MAX_EXPOSURE; 
	int laser_tests_passed = 0;
	bool self_test_passed = false, self_test_failed = false;
	tx_offsets offsets;
	offsets.x = TX_OFFSET_X_DEFAULT; offsets.y = TX_OFFSET_Y_DEFAULT;
	calculateTxOffsets(pat_health_port, textFileOut, fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer, offsets); 
	int dither_count = 0; bool dithering_on = false; float offset_x_init, offset_y_init;
	float period_calculate_tx_offsets = PERIOD_CALCULATE_TX_OFFSETS;
	float period_dither_tx_offsets = PERIOD_DITHER_TX_OFFSETS;

	offsetParamStruct offsetParams[NUM_TX_OFFSET_PARAMS]; //array of offsetParamStruct
    std::cout << getOffsetParams(offsetParams) << std::endl; //get tx offset parameters from csv file
	
	//set up self test error buffer
	std::stringstream self_test_stream;
	char self_test_error_buffer[BUFFER_SIZE];	
	self_test_stream.rdbuf()->pubsetbuf(self_test_error_buffer, sizeof(self_test_error_buffer));

	// Killing app handler (Enables graceful Ctrl+C exit)
	signal(SIGINT, [](int signum) { stop = true; });

	// Hardware init				
	Camera camera(textFileOut, pat_health_port);	
	//Catch camera initialization failure state in a re-initialization loop:
	bool camera_initialized = camera.initialize();	
	while(!camera_initialized){
		send_packet_pat_status(pat_status_port, STATUS_CAMERA_INIT);
		log(pat_health_port, textFileOut, "In main.cpp - Camera Init - Camera Initialization Failed! Error:", camera.error);
		// Listen for exit command 		
		zmq::poll(poll_pat_control.data(), 1, period_hb_tlm_ms); // when timeout_ms (the third argument here) is -1, then block until ready to receive (based on: https://ogbe.net/blog/zmq_helloworld.html)
		if(poll_pat_control[0].revents & ZMQ_POLLIN){
			// received something on the first (only) socket
			command = receive_packet_pat_control(pat_control_port);
			if(command == CMD_START_PAT_STATIC_POINT){
				log(pat_health_port, textFileOut, "In main.cpp - Camera Init - Received CMD_START_PAT_STATIC_POINT command. Proceeding to main PAT loop...");
				phase = STATIC_POINT; 
				STANDBY = false;
				break;
			} else if(command == CMD_SELF_TEST){
				log(pat_health_port, textFileOut, "In main.cpp - Camera Init - Received CMD_SELF_TEST command.");	
				camera_test_result = FAIL_SELF_TEST; 
				//stream error message to buffer
				self_test_stream << "Camera Initialization Failed! Error:" << camera.error;
				if(check_fpga_comms(fpga_map_answer_port, poll_fpga_answer, fpga_map_request_port)){
					fpga_test_result = PASS_SELF_TEST;
					log(pat_health_port, textFileOut, "In main.cpp - Camera Init - CMD_SELF_TEST - FPGA test passed.");
				} else{
					fpga_test_result = FAIL_SELF_TEST;
					log(pat_health_port, textFileOut, "In main.cpp - Camera Init - CMD_SELF_TEST - FPGA test failed.");
					self_test_stream << "FPGA Comms Fault\n"; //stream error message to buffer
				}
				laser_test_result = NULL_SELF_TEST;
				fsm_test_result = NULL_SELF_TEST;
				calibration_test_result = NULL_SELF_TEST;			
				//send self test results
				send_packet_self_test(tx_packets_port, camera_test_result, fpga_test_result, laser_test_result, fsm_test_result, calibration_test_result, self_test_error_buffer);
				
			} else if(command == CMD_END_PROCESS){
				log(pat_health_port, textFileOut, "In main.cpp - Camera Init - Received CMD_END_PROCESS command. Saving text file and ending process.");
				textFileOut.close(); //close telemetry text file
				pat_status_port.close();
				pat_health_port.close();
				pat_control_port.close();
				fpga_map_request_port.close();
				fpga_map_answer_port.close();
				tx_packets_port.close();
				context.close();
				exit(0); 
			}
		}
		//Try to initialize again
		camera_initialized = camera.initialize();

		// Allow graceful exit with Ctrl-C
		if(stop){
			log(pat_health_port, textFileOut, "In main.cpp - Camera Init - Saving text file and ending process.");
			textFileOut.close(); //close telemetry text file
			pat_status_port.close();
			pat_health_port.close();
			pat_control_port.close();
			fpga_map_request_port.close();
			fpga_map_answer_port.close();
			tx_packets_port.close();
			context.close();
			exit(0); 
		}
	}
	if(camera_initialized){log(pat_health_port, textFileOut, "In main.cpp Camera Connection Initialized");}
	
	FSM fsm(textFileOut, pat_health_port, fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer);
	Calibration calibration(camera, fsm, textFileOut, pat_health_port);
	Tracking track(camera, calibration, textFileOut, pat_status_port, pat_health_port, pat_control_port, poll_pat_control);
	std::this_thread::sleep_for(std::chrono::seconds(1));

	// CSV data saving for main PAT loop
	time_point<steady_clock> beginTime;
	map<double, CSVdata> csvData;
	bool haveCsvData = false; //don't save csv if there's no data

	//Beacon Loss Timing
	time_point<steady_clock> startBeaconLoss;
	duration<double> waitBeaconLoss(PERIOD_BEACON_LOSS); //wait time before switching back to ACQUISITION

	//Calib Loss Timing
	time_point<steady_clock> startCalibLoss;
	duration<double> waitCalibLoss(PERIOD_CALIB_LOSS); //wait time before switching back to ACQUISITION

	//Health Heartbeat Telemetry Timing
	time_point<steady_clock> time_prev_heartbeat;
	duration<double> period_heartbeat(PERIOD_HEARTBEAT_TLM); //wait time in between heartbeat messages (0.5s = 2 Hz)
	time_point<steady_clock> check_heartbeat; // Record current time
	duration<double> elapsed_time_heartbeat; // time since tx adcs tlm

	//CSV Telemetry Timing
	time_point<steady_clock> time_prev_csv_write; 
	duration<double> period_csv_write(PERIOD_CSV_WRITE); //wait time in between csv writes (0.1s = 10 Hz)
	time_point<steady_clock> check_csv_write; // Record current time
	duration<double> elapsed_time_csv_write; // time since tx adcs tlm
	time_point<steady_clock> now;
	duration<double> diff;

	//Bus feedback Telemetry Timing
	time_point<steady_clock> time_prev_tx_adcs; 
	duration<double> period_tx_adcs(PERIOD_TX_ADCS); //wait time in between feedback messages to the bus (1s = 1 Hz)
	time_point<steady_clock> check_tx_adcs; // Record current time
	duration<double> elapsed_time_tx_adcs; // time since tx adcs tlm

	//Tx Offset Calculation Timing
	time_point<steady_clock> time_prev_tx_offset; 
	time_point<steady_clock> check_tx_offset; // Record current time
	duration<double> elapsed_time_tx_offset; // time since last tx offset calculation

	//Tx Offset Dithering Timing
	time_point<steady_clock> time_prev_dither; 
	time_point<steady_clock> check_dither; // Record current time
	duration<double> elapsed_time_dither; // time since last tx offset calculation
	
	// Enter Primary Process Loop: Standby + Main
	while(OPERATIONAL){
		// Allow graceful exit with Ctrl-C
		if(stop){break;}

		// START Standby Loop
		while(STANDBY){
			// Allow graceful exit with Ctrl-C
			if(stop){
				OPERATIONAL = false;
				break;
			}
			if(self_test_passed){
				send_packet_pat_status(pat_status_port, STATUS_STANDBY_SELF_TEST_PASSED); //status message
			} else if(self_test_failed){
				send_packet_pat_status(pat_status_port, STATUS_STANDBY_SELF_TEST_FAILED); //status message
			} else if(haveCalibKnowledge){
				send_packet_pat_status(pat_status_port, STATUS_STANDBY_CALIBRATED); //status message
			} else {
				send_packet_pat_status(pat_status_port, STATUS_STANDBY); //status message
			}
			if(haveCalibKnowledge){
				log(pat_health_port, textFileOut, "In main.cpp - Standby - Calib is at [", calib.x, ",", calib.y, ", valueMax = ", calib.valueMax, ", valueSum = ", calib.valueSum, ", pixelCount = ", calib.pixelCount, "]");
			}
			log(pat_health_port, textFileOut, "In main.cpp - Standby - Standing by for command..."); //health heartbeat	
			// Listen for command 		
			zmq::poll(poll_pat_control.data(), 1, period_hb_tlm_ms); // when timeout_ms (the third argument here) is -1, then block until ready to receive (based on: https://ogbe.net/blog/zmq_helloworld.html)
			if(poll_pat_control[0].revents & ZMQ_POLLIN)
			{
				// received something on the first (only) socket
				char command_data[CMD_PAYLOAD_SIZE];
				command = receive_packet_pat_control(pat_control_port, command_data);
				switch(command)
				{
					case CMD_START_PAT:
						main_entry_flag = atoi(command_data); 
						if(main_entry_flag == SKIP_CALIB_FLAG){
							log(pat_health_port, textFileOut, "In main.cpp - Standby - Received CMD_START_PAT command with skip calibration flag. Proceeding to ACQUISITION...");
							if(haveCalibKnowledge){phase = ACQUISITION;}
							else{
								log(pat_health_port, textFileOut, "In main.cpp - Standby - Do not have calibration knowledge. Proceeding to calibration.");
								phase = CALIBRATION;
							}
							STANDBY = false;
						} else if(main_entry_flag == DO_CALIB_FLAG){
							log(pat_health_port, textFileOut, "In main.cpp - Standby - Received CMD_START_PAT command with do calibration flag. Proceeding to CALIBRATION...");
							phase = CALIBRATION;
							STANDBY = false;
						} else{
							log(pat_health_port, textFileOut, "In main.cpp - Standby - Received CMD_START_PAT command in unknown configuration. Standing by...");
						}
						calculateTxOffsets(pat_health_port, textFileOut, fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer, offsets);
						break;
						
					case CMD_START_PAT_OPEN_LOOP:
						main_entry_flag = atoi(command_data); 
						if(main_entry_flag == SKIP_CALIB_FLAG){
							log(pat_health_port, textFileOut, "In main.cpp - Standby - Received CMD_START_PAT_OPEN_LOOP command with skip calibration flag. Proceeding to ACQUISITION...");
							openLoop = true;
							if(haveCalibKnowledge){phase = ACQUISITION;}
							else{
								log(pat_health_port, textFileOut, "In main.cpp - Standby - Do not have calibration knowledge. Proceeding to calibration.");
								phase = CALIBRATION;
							}
							STANDBY = false;
						} else if(main_entry_flag == DO_CALIB_FLAG){
							log(pat_health_port, textFileOut, "In main.cpp - Standby - Received CMD_START_PAT_OPEN_LOOP command with do calibration flag. Proceeding to CALIBRATION...");
							openLoop = true;
							phase = CALIBRATION;
							STANDBY = false;
						} else{
							log(pat_health_port, textFileOut, "In main.cpp - Standby - Received CMD_START_PAT_OPEN_LOOP command in unknown configuration. Standing by...");
						}
						calculateTxOffsets(pat_health_port, textFileOut, fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer, offsets);
						break;
					
					case CMD_START_PAT_STATIC_POINT:
						log(pat_health_port, textFileOut, "In main.cpp - Standby - Received CMD_START_PAT_STATIC_POINT command. Proceeding to main PAT loop...");
						phase = STATIC_POINT;
						STANDBY = false;
						calculateTxOffsets(pat_health_port, textFileOut, fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer, offsets);
						break;
						
					case CMD_START_PAT_BUS_FEEDBACK:
						main_entry_flag = atoi(command_data); 
						if(main_entry_flag == SKIP_CALIB_FLAG){
							log(pat_health_port, textFileOut, "In main.cpp - Standby - Received CMD_START_PAT_BUS_FEEDBACK command with skip calibration flag. Proceeding to ACQUISITION...");
							sendBusFeedback = true;
							if(haveCalibKnowledge){phase = ACQUISITION;}
							else{
								log(pat_health_port, textFileOut, "In main.cpp - Standby - Do not have calibration knowledge. Proceeding to calibration.");
								phase = CALIBRATION;
							}
							STANDBY = false;
						} else if(main_entry_flag == DO_CALIB_FLAG){
							log(pat_health_port, textFileOut, "In main.cpp - Standby - Received CMD_START_PAT_BUS_FEEDBACK command with do calibration flag Proceeding to CALIBRATION...");
							sendBusFeedback = true;
							phase = CALIBRATION;
							STANDBY = false;
						} else{
							log(pat_health_port, textFileOut, "In main.cpp - Standby - Received CMD_START_PAT_BUS_FEEDBACK command in unknown configuration. Standing by...");
						}
						calculateTxOffsets(pat_health_port, textFileOut, fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer, offsets);
						break;

					case CMD_START_PAT_OPEN_LOOP_BUS_FEEDBACK:
						main_entry_flag = atoi(command_data); 
						if(main_entry_flag == SKIP_CALIB_FLAG){
							log(pat_health_port, textFileOut, "In main.cpp - Standby - Received CMD_START_PAT_OPEN_LOOP_BUS_FEEDBACK command with skip calibration flag. Proceeding to ACQUISITION...");
							openLoop = true; 
							sendBusFeedback = true;
							if(haveCalibKnowledge){phase = ACQUISITION;}
							else{
								log(pat_health_port, textFileOut, "In main.cpp - Standby - Do not have calibration knowledge. Proceeding to calibration.");
								phase = CALIBRATION;
							}
							STANDBY = false;
						} else if(main_entry_flag == DO_CALIB_FLAG){
							log(pat_health_port, textFileOut, "In main.cpp - Standby - Received CMD_START_PAT_OPEN_LOOP_BUS_FEEDBACK command with do calibration flag. Proceeding to CALIBRATION...");
							openLoop = true; 
							sendBusFeedback = true;
							phase = CALIBRATION;
							STANDBY = false;
						} else{
							log(pat_health_port, textFileOut, "In main.cpp - Standby - Received CMD_START_PAT_OPEN_LOOP_BUS_FEEDBACK command in unknown configuration. Standing by...");
						}
						calculateTxOffsets(pat_health_port, textFileOut, fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer, offsets);
						break;	

					case CMD_SET_GET_IMAGE_CENTER_X:
						cmd_get_image_center_x_rel = atoi(command_data); 
						if(abs(cmd_get_image_center_x_rel) <= CAMERA_WIDTH/2 - getImageWindow.w/2){
							getImageWindow.x = CAMERA_WIDTH/2 + cmd_get_image_center_x_rel - getImageWindow.w/2;
							log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_SET_GET_IMAGE_CENTER_X - Updating Get Image Window Center X to ", cmd_get_image_center_x_rel, " rel-to-ctr.");
						} else{
							log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_SET_GET_IMAGE_CENTER_X - Error, command out of bounds: ", cmd_get_image_center_x_rel);
						}
						break;

					case CMD_SET_GET_IMAGE_CENTER_Y:
						cmd_get_image_center_y_rel = atoi(command_data); 
						if(abs(cmd_get_image_center_y_rel) <= CAMERA_HEIGHT/2 - getImageWindow.h/2){
							getImageWindow.y = CAMERA_HEIGHT/2 + cmd_get_image_center_y_rel - getImageWindow.h/2;
							log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_SET_GET_IMAGE_CENTER_Y - Updating Get Image Window Center Y to ", cmd_get_image_center_y_rel, " rel-to-ctr.");
						} else{
							log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_SET_GET_IMAGE_CENTER_Y - Error, command out of bounds: ", cmd_get_image_center_y_rel);
						}
						break;

					case CMD_SET_GET_IMAGE_WINDOW_WIDTH:
						cmd_get_image_w = atoi(command_data); 
						if(cmd_get_image_w <= CAMERA_WIDTH){
							getImageWindow.w = cmd_get_image_w;
							log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_SET_GET_IMAGE_WINDOW_WIDTH - Updating Get Image Window Width to ", getImageWindow.w);
						} else{
							log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_SET_GET_IMAGE_WINDOW_WIDTH - Error, command out of bounds: ", getImageWindow.w);
						}
						break;	

					case CMD_SET_GET_IMAGE_WINDOW_HEIGHT:
						cmd_get_image_h = atoi(command_data); 
						if(cmd_get_image_h <= CAMERA_HEIGHT){
							getImageWindow.h = cmd_get_image_h;
							log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_SET_GET_IMAGE_WINDOW_HEIGHT - Updating Get Image Window Height to ", getImageWindow.h);
						} else{
							log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_SET_GET_IMAGE_WINDOW_HEIGHT - Error, command out of bounds: ", getImageWindow.h);
						}
						break;	
						
					case CMD_GET_IMAGE:
						//set to commanded exposure
						command_exposure = atoi(command_data); 
						if(command_exposure < MIN_EXPOSURE){command_exposure = MIN_EXPOSURE;}
						if(command_exposure > MAX_EXPOSURE){command_exposure = MAX_EXPOSURE;}
						log(pat_health_port, textFileOut, "In main.cpp - Standby - Received CMD_GET_IMAGE command with exposure = ", command_exposure);
						camera.config->expose_us.write(command_exposure);

						//set window size
						camera.setWindow(getImageWindow);

						//save image
						logImage(string("CMD_GET_IMAGE"), camera, textFileOut, pat_health_port); 
						break;		
								
					case CMD_CALIB_TEST:
						log(pat_health_port, textFileOut, "In main.cpp - Standby - Received CMD_CALIB_TEST command.");
						//Run calibration:
						if(laserOn(pat_health_port, textFileOut, fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer)){ //turn calibration laser on	
							if(calibration.run(calib)) //sets calib exposure
							{
								calibExposure = camera.config->expose_us.read(); //save calib exposure
								log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_CALIB_TEST - Calibration complete. Calib Exposure = ", calibExposure, " us.");
								haveCalibKnowledge = true;
							}
							else
							{
								log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_CALIB_TEST - Calibration failed!");
								haveCalibKnowledge = false;
							}					
						} else{
							log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_CALIB_TEST - laserOn FPGA command failed!");
						}
						break;
					
					case CMD_CALIB_LASER_TEST:
						//set to commanded exposure
						command_exposure = atoi(command_data); 
						if(command_exposure < MIN_EXPOSURE){command_exposure = MIN_EXPOSURE;}
						if(command_exposure > MAX_EXPOSURE){command_exposure = MAX_EXPOSURE;}
						log(pat_health_port, textFileOut, "In main.cpp - Standby - Received CMD_CALIB_LASER_TEST command with exposure = ", command_exposure);
						camera.config->expose_us.write(command_exposure);		
						
						//set to sufficiently large window size (but not too large)
						camera.setCenteredWindow(CAMERA_WIDTH/2, CAMERA_HEIGHT/2, CALIB_BIG_WINDOW);

						//ensure FSM is centered
						fsm.setNormalizedAngles(0,0); 
						for(int i = 0; i < 2; i++){ //run twice to make sure on/off switching is working
							//switch laser on
							if(laserOn(pat_health_port, textFileOut, fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer, i)){
								if(calibration.checkLaserOn(calib)){
									log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_CALIB_LASER_TEST - laserOn check passed.");
									logImage(string("CMD_CALIB_LASER_TEST_ON"), camera, textFileOut, pat_health_port); //save image
									if(laserOff(fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer, i)){
										if(calibration.checkLaserOff()){
											log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_CALIB_LASER_TEST - laserOff check passed.");
											logImage(string("CMD_CALIB_LASER_TEST_OFF"), camera, textFileOut, pat_health_port);
										} else{
											log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_CALIB_LASER_TEST - laserOff check failed!");
										}
									} else{
										log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_CALIB_LASER_TEST - laserOff FPGA command failed!");
									}
								} else{
									log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_CALIB_LASER_TEST - laserOn check failed!");
								}
							} else{
								log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_CALIB_LASER_TEST - laserOn FPGA command failed!");
							}	
						}
						break;

					case CMD_FSM_TEST:
						//set to commanded exposure
						command_exposure = atoi(command_data); 
						if(command_exposure < MIN_EXPOSURE){command_exposure = MIN_EXPOSURE;}
						if(command_exposure > MAX_EXPOSURE){command_exposure = MAX_EXPOSURE;}
						log(pat_health_port, textFileOut, "In main.cpp - Standby - Received CMD_FSM_TEST command with exposure = ", command_exposure);
						camera.config->expose_us.write(command_exposure);		

						//set to sufficiently large window size (but not too large)
						camera.setCenteredWindow(CAMERA_WIDTH/2, CAMERA_HEIGHT/2, CALIB_BIG_WINDOW);	

						//switch laser on
						if(laserOn(pat_health_port, textFileOut, fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer)){
							if(calibration.testFSM(calib)){
								fsm_test_result = PASS_SELF_TEST;
								log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_FSM_TEST - FSM test passed.");
							} else{
								log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_FSM_TEST - FSM test failed.");
							}
						} else{
							log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_FSM_TEST - laserOn FPGA command failed!");
						}
						break;

					case CMD_BCN_ALIGN:
						log(pat_health_port, textFileOut, "In main.cpp - Standby - Received CMD_BCN_ALIGN command. Proceeding to main PAT loop...");	
						phase = ACQUISITION; //skip calibration for beacon alignment with GSE
						bcnAlignment = true; //ignore laser off commands and skip open-loop fsm commands
						openLoop = true; //transition to open-loop pointing after acquisition for alignment
						STANDBY = false;
						break;

					case CMD_SET_BEACON_X:
						beacon_x_rel = atoi(command_data); 
						beacon.x = beacon_x_rel + CAMERA_WIDTH/2;
						log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_SET_BEACON_X - Updating Beacon X to ", beacon_x_rel, " rel to center =>, ", beacon.x, " absolute");
						break;

					case CMD_SET_BEACON_Y:
						beacon_y_rel = atoi(command_data); 
						beacon.y = beacon_y_rel + CAMERA_HEIGHT/2;
						log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_SET_BEACON_Y - Updating Beacon Y to ", beacon_y_rel, " rel to center =>, ", beacon.y, " absolute");
						break;

					case CMD_SET_BEACON_WINDOW_SIZE:
						beaconWindowSize = atoi(command_data); 
						beaconWindow.w = beaconWindowSize;
						beaconWindow.h = beaconWindowSize;
						log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_SET_BEACON_WINDOW_SIZE - Updating Beacon Window Size to ", beaconWindowSize);
						break;

					case CMD_SET_BEACON_MAX_EXP:
						maxBcnExposure = atoi(command_data); 
						log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_SET_BEACON_MAX_EXP - Updating Beacon Max Exposure (us) to ", maxBcnExposure);
						break;

					case CMD_TX_ALIGN:
						if(haveCalibKnowledge){
							log(pat_health_port, textFileOut, "In main.cpp - Standby - Executing CMD_TX_ALIGN command.");
							double setPointX_OL = (double) CAMERA_WIDTH/2 + (double) offsets.x;
							double setPointY_OL = (double) CAMERA_HEIGHT/2 + (double) offsets.y;
							calib.x = (int) setPointX_OL;
							calib.y = (int) setPointY_OL;
							track.controlOpenLoop(fsm, setPointX_OL, setPointY_OL);
						} else{
							log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_TX_ALIGN - Do not have calibration knowledge. Run CMD_CALIB_TEST first.");
						}
						break;

					case CMD_UPDATE_TX_OFFSET_X:
						offsets.x = atoi(command_data); 
						log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_UPDATE_TX_OFFSET_X - Updating Tx Offset X to ", offsets.x);
						break;

					case CMD_UPDATE_TX_OFFSET_Y:
						offsets.y = atoi(command_data); 
						log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_UPDATE_TX_OFFSET_Y - Updating Tx Offset Y to ", offsets.y);
						break;

					case CMD_UPDATE_PERIOD_CALCULATE_TX_OFFSET:
						period_calculate_tx_offsets = (float) atoi(command_data); 
						log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_UPDATE_PERIOD_CALCULATE_TX_OFFSET - Updating Tx Offset Calculation PD to ", period_calculate_tx_offsets);
						break;

					case CMD_ENABLE_DITHER_TX_OFFSET:
						dithering_on = true;
						log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_ENABLE_DITHER_TX_OFFSET - Tx Offset dithering enabled.");
						break;

					case CMD_UPDATE_PERIOD_DITHER_TX_OFFSET:
						period_dither_tx_offsets = (float) atoi(command_data); 
						log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_UPDATE_PERIOD_DITHER_TX_OFFSET - Updating Tx Offset Dither PD to ", period_dither_tx_offsets);
						break;

					case CMD_UPDATE_FSM_X:
						command_offset_x = atoi(command_data); 
						if(haveCalibKnowledge){
							log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_UPDATE_FSM_X - Offsetting X by ", command_offset_x, " pixels");
							calib.x += command_offset_x;
							track.controlOpenLoop(fsm, calib.x, calib.y);
						} else{
							log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_UPDATE_FSM_X - Do not have calibration knowledge. Run CMD_CALIB_TEST first."); 
						}
						break;

					case CMD_UPDATE_FSM_Y:
						command_offset_y = atoi(command_data); 
						if(haveCalibKnowledge){
							log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_UPDATE_FSM_Y - Offsetting Y by ", command_offset_y, " pixels");
							calib.y += command_offset_y;
							track.controlOpenLoop(fsm, calib.x, calib.y);
						} else{
							log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_UPDATE_FSM_Y - Do not have calibration knowledge. Run CMD_CALIB_TEST first."); 
						}
						break;

					case CMD_SELF_TEST:
						log(pat_health_port, textFileOut, "In main.cpp - Standby - Received CMD_SELF_TEST command.");
						//Camera Test:
						if(camera_initialized){
							camera_test_result = PASS_SELF_TEST;
							log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_SELF_TEST - Camera test passed.");
						} else{
							camera_test_result = FAIL_SELF_TEST;
							log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_SELF_TEST - Camera test failed!");
							self_test_stream << "Camera Fault\n"; //stream error message to buffer
						}

						//FPGA Comms Test:
						if(check_fpga_comms(fpga_map_answer_port, poll_fpga_answer, fpga_map_request_port)){
							fpga_test_result = PASS_SELF_TEST;
							log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_SELF_TEST - FPGA test passed.");
						} else{
							fpga_test_result = FAIL_SELF_TEST;
							log(pat_health_port, textFileOut, "In main.cpp - Standby - CMD_SELF_TEST - FPGA test failed.");
							self_test_stream << "FPGA Comms Fault\n"; //stream error message to buffer
						}						
						
						//Laser Test:
						if((camera_test_result = PASS_SELF_TEST) && (fpga_test_result == PASS_SELF_TEST)){							
							fsm.setNormalizedAngles(0,0); //ensure FSM is centered
							if(laserOn(pat_health_port, textFileOut, fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer, 0)){
								if(calibration.findExposureRange(true)){
									calibExposure = calibration.preferredExpo;
									log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_SELF_TEST - (Laser Test) Using auto-tuned exposure = ", calibExposure, " us.");
								} else{
									calibExposure = CALIB_EXPOSURE_SELF_TEST; 	
									log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_SELF_TEST - (Laser Test) Setting to default calib exposure = ", CALIB_EXPOSURE_SELF_TEST, " us.");
								}
								camera.config->expose_us.write(calibExposure); //set calib exposure
								camera.setCenteredWindow(CAMERA_WIDTH/2, CAMERA_HEIGHT/2, CALIB_BIG_WINDOW); //set to sufficiently large window size (but not too large)							 
								laser_tests_passed = 0;
								for(int i = 1; i < 3; i++){ //run twice to make sure on/off switching is working
									if(laserOn(pat_health_port, textFileOut, fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer, i)){
										if(calibration.checkLaserOn(calib)){
											log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_SELF_TEST - (Laser Test) laserOn check passed.");
											logImage(string("CMD_SELF_TEST_LASER_ON_") + to_string(i), camera, textFileOut, pat_health_port); //save image
											if(laserOff(fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer, i)){
												if(calibration.checkLaserOff()){
													log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_SELF_TEST - (Laser Test) laserOff check passed.");
													logImage(string("CMD_SELF_TEST_LASER_OFF_") + to_string(i), camera, textFileOut, pat_health_port);
													laser_tests_passed++;
												} else{
													log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_SELF_TEST - (Laser Test) laserOff check failed!");
													self_test_stream << "(Laser Test) laserOff check failed!\n";
													logImage(string("CMD_SELF_TEST_LASER_OFF_") + to_string(i), camera, textFileOut, pat_health_port);
												}
											} else{
												log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_SELF_TEST - (Laser Test) laserOff FPGA command failed!");
												self_test_stream << "(Laser Test) laserOff FPGA command failed!\n";
											}
										} else{
											log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_SELF_TEST - (Laser Test) laserOn check failed!");
											self_test_stream << "(Laser Test) laserOn check failed!\n";
											logImage(string("CMD_SELF_TEST_LASER_ON_") + to_string(i), camera, textFileOut, pat_health_port); //save image
										}
									} else{
										log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_SELF_TEST - (Laser Test) laserOn FPGA command failed!");
										self_test_stream << "(Laser Test) laserOn FPGA command failed!\n";
									}	
								}
							} else{
								log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_SELF_TEST - (Laser Test) laserOn FPGA command failed!");
								self_test_stream << "(Laser Test) laserOn FPGA command failed!\n";
							}
							if(laser_tests_passed == 2){laser_test_result = PASS_SELF_TEST;}
							else{laser_test_result = FAIL_SELF_TEST;}								

							if(laser_test_result == PASS_SELF_TEST){
								//FSM Test:
								if(laserOn(pat_health_port, textFileOut, fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer)){ //turn calibration laser on
									if(calibration.testFSM(calib)){
										fsm_test_result = PASS_SELF_TEST;
										log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_SELF_TEST - FSM test passed.");
									} else{
										log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_SELF_TEST - FSM test failed.");
										self_test_stream << "(FSM Test) FSM test failed.\n"; //stream error message to buffer
										fsm_test_result = FAIL_SELF_TEST;
									}
								} else{
									log(pat_health_port, textFileOut,  "In main.cpp - Standby - CMD_SELF_TEST - laserOn FPGA command failed!");
									self_test_stream << "(FSM Test) laserOn FPGA command failed!\n"; //stream error message to buffer
									fsm_test_result = NULL_SELF_TEST;
								}
							
								if(fsm_test_result == PASS_SELF_TEST){
									//Calibration Test:
									if(calibration.run(calib)) //sets calib exposure
									{
										calibExposure = camera.config->expose_us.read(); //save calib exposure
										log(pat_health_port, textFileOut, "In main.cpp CMD_SELF_TEST - (Calibration Test). Calib Exposure = ", calibExposure, " us.");
										haveCalibKnowledge = true; 
										//Check offset
										if(abs(calibration.centerOffsetX) <= CALIB_OFFSET_TOLERANCE){
											if(abs(calibration.centerOffsetY) <= CALIB_OFFSET_TOLERANCE){
												if(calibration.s00/calibration.s11 - 1/sqrt(2) <= CALIB_SENSITIVITY_RATIO_TOL){
													calibration_test_result = PASS_SELF_TEST;
													self_test_passed = true;
													log(pat_health_port, textFileOut, "In main.cpp CMD_SELF_TEST - Calibration Test passed.");
													self_test_stream << "None";
												} else{
													log(pat_health_port, textFileOut, "In main.cpp CMD_SELF_TEST - Sensitivity ratio check failed.");
													self_test_stream << "(Calibration Test) Sensitivity ratio check failed.\n";
													calibration_test_result = FAIL_SELF_TEST;
												}
											} else{
												log(pat_health_port, textFileOut, "In main.cpp CMD_SELF_TEST - (Calibration Test) Y offset check failed: (calibration.centerOffsetY = ", calibration.centerOffsetY, ") > (CALIB_OFFSET_TOLERANCE = ", CALIB_OFFSET_TOLERANCE, ")"); 
												self_test_stream << "(Calibration Test) Y offset (" << calibration.centerOffsetY << ") check failed.\n";
												calibration_test_result = FAIL_SELF_TEST;
											}
										} else{
											log(pat_health_port, textFileOut, "In main.cpp CMD_SELF_TEST - (Calibration Test) X offset check failed: (calibration.centerOffsetX = ", calibration.centerOffsetX, ") > (CALIB_OFFSET_TOLERANCE = ", CALIB_OFFSET_TOLERANCE, ")"); 
											self_test_stream << "(Calibration Test) X offset (" << calibration.centerOffsetX << ") check failed.\n";
											calibration_test_result = FAIL_SELF_TEST;
										}
									} else{
										log(pat_health_port, textFileOut,  "In main.cpp CMD_SELF_TEST - (Calibration Test) Calibration failed.");
										self_test_stream << "(Calibration Test) Calibration failed.\n"; //stream error message to buffer
										calibration_test_result = FAIL_SELF_TEST;
									}	
								} else{		
									calibration_test_result = NULL_SELF_TEST;
								}
							} else{
								fsm_test_result = NULL_SELF_TEST;
								calibration_test_result = NULL_SELF_TEST;
							}						
						} else{
							laser_test_result = NULL_SELF_TEST;
							fsm_test_result = NULL_SELF_TEST;
							calibration_test_result = NULL_SELF_TEST;
						}
						if(!self_test_passed){self_test_failed = true;}
						//send self test results
						send_packet_self_test(tx_packets_port, camera_test_result, fpga_test_result, laser_test_result, fsm_test_result, calibration_test_result, self_test_error_buffer);
						break;

					case CMD_END_PROCESS:
						log(pat_health_port, textFileOut, "In main.cpp - Standby - Received CMD_END_PROCESS command. Exiting...");
						STANDBY = false;
						OPERATIONAL = false;
						break;
						
					default:
						log(pat_health_port, textFileOut, "In main.cpp - Standby - Received unknown command: ", command);
				}	
			}
		}		
		// END of STANDBY loop
		duration<double> period_tx_offset(period_calculate_tx_offsets); //wait time in between calculation of tx offsets
		duration<double> period_dither(period_dither_tx_offsets); //wait time in between dithering
		
		// START Main PAT Loop
		while(OPERATIONAL && !STANDBY) 
		{	
			// Allow graceful exit with Ctrl-C
			if(stop){
				OPERATIONAL = false;
				break;
			}
			
			// Listen for Commands
			zmq::poll(poll_pat_control.data(), 1, 10); // when timeout_ms (the third argument here) is -1, then block until ready to receive (based on: https://ogbe.net/blog/zmq_helloworld.html)
			if(poll_pat_control[0].revents & ZMQ_POLLIN) {
				// received something on the first (only) socket
				char command_data[CMD_PAYLOAD_SIZE];
				command = receive_packet_pat_control(pat_control_port, command_data);
				if (command == CMD_END_PAT){
					break;
				} else if(command == CMD_END_PROCESS){
					OPERATIONAL = false;
					break; 
				} else if(command == CMD_UPDATE_TX_OFFSET_X){
					offsets.x = atoi(command_data); 
					log(pat_health_port, textFileOut, "In main.cpp phase ", phaseNames[phase]," - Updating Tx Offset X to ", offsets.x);
				} else if(command == CMD_UPDATE_TX_OFFSET_Y){
					offsets.y = atoi(command_data); 
					log(pat_health_port, textFileOut, "In main.cpp phase ", phaseNames[phase]," - Updating Tx Offset Y to ", offsets.y);
				}
			}
			if(track.received_end_pat_cmd){
				break;
			} else if(track.received_end_process_cmd){
				OPERATIONAL = false;
				break; 
			}
			
			check_heartbeat = steady_clock::now(); // Record current time
			elapsed_time_heartbeat = check_heartbeat - time_prev_heartbeat; // Calculate time since heartbeat tlm
			if(elapsed_time_heartbeat > period_heartbeat) //pg
			{
				send_packet_pat_status(pat_status_port, STATUS_MAIN); //send status message
				if(phase == OPEN_LOOP){
					if(haveBeaconKnowledge){
						log(pat_health_port, textFileOut, "In main.cpp phase ", phaseNames[phase]," - Beacon is at [", beacon.x - CAMERA_WIDTH/2, ",", beacon.y - CAMERA_HEIGHT/2, " rel-to-center, exp = ", beaconExposure, "valueMax = ", beacon.valueMax, "valueSum = ", beacon.valueSum, "pixelCount = ", beacon.pixelCount, "]");
					} else{
						log(pat_health_port, textFileOut, "In main.cpp phase ", phaseNames[phase]," - No idea where beacon is.");
					}
				} else{
					if(beaconExposure == TRACK_MIN_EXPOSURE) log(pat_health_port, textFileOut,  "In main.cpp console update - Minimum beacon exposure reached!"); //notification when exposure limits reached, pg
					if(beaconExposure == TRACK_MAX_EXPOSURE) log(pat_health_port, textFileOut,  "In main.cpp console update - Maximum beacon exposure reached!");
					if(haveBeaconKnowledge){
						log(pat_health_port, textFileOut, "In main.cpp phase ", phaseNames[phase]," - Beacon is at [", beacon.x - CAMERA_WIDTH/2, ",", beacon.y - CAMERA_HEIGHT/2, " rel-to-center, exp = ", beaconExposure, "valueMax = ", beacon.valueMax, "valueSum = ", beacon.valueSum, "pixelCount = ", beacon.pixelCount, "]");
					} else{
						log(pat_health_port, textFileOut, "In main.cpp phase ", phaseNames[phase]," - No idea where beacon is.");
					}
					if(haveCalibKnowledge){
						log(pat_health_port, textFileOut, "In main.cpp phase ", phaseNames[phase]," - Calib is at [", calib.x - CAMERA_WIDTH/2, ",", calib.y - CAMERA_HEIGHT/2, " rel-to-center, exp = ", calibExposure, ", valueMax = ", calib.valueMax, ", valueSum = ", calib.valueSum, ", pixelCount = ", calib.pixelCount, "]");
					} else{
						log(pat_health_port, textFileOut, "In main.cpp phase ", phaseNames[phase]," - No idea where calib is.");
					}
				}	
				time_prev_heartbeat = steady_clock::now(); // Record time of message		
			}

			check_tx_offset = steady_clock::now(); // Record current time
			elapsed_time_tx_offset = check_tx_offset - time_prev_tx_offset; // Calculate time since last tx offset calculation
			if(elapsed_time_tx_offset > period_tx_offset){
				log(pat_health_port, textFileOut, "In main.cpp - MAIN - phase ", phaseNames[phase]," - Updating Tx offsets.");
				calculateTxOffsets(pat_health_port, textFileOut, fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer, offsets);
				time_prev_tx_offset = steady_clock::now();
				if(dithering_on){
					offset_x_init = offsets.x; offset_y_init = offsets.y; //update offsets for dithering
					dither_count = 0; //reset dither counter
				} 
			}
	
			//PAT Phases:		
			switch(phase)
			{
				// Calibration phase, internal laser has to be turned on, ran just once
				case CALIBRATION:
					if(laserOn(pat_health_port, textFileOut, fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer)){ //turn calibration laser on
						if(calibration.run(calib)) //sets calib exposure, pg-comment
						{
							haveCalibKnowledge = true; 
							calibExposure = camera.config->expose_us.read(); //save calib exposure, pg
							log(pat_health_port, textFileOut, "In main.cpp phase CALIBRATION - Calibration complete. Calib is at [", calib.x, ",", calib.y, ", exp = ", calibExposure, ", valueMax = ", calib.valueMax, ", valueSum = ", calib.valueSum, ", pixelCount = ", calib.pixelCount, "]");
							phase = ACQUISITION;
						}
						else
						{
							haveCalibKnowledge = false; 
							log(pat_health_port, textFileOut,  "In main.cpp phase CALIBRATION - calibration.run failed!");
							num_calibration_attempts++;
							log(pat_health_port, textFileOut,  "In main.cpp phase CALIBRATION - Calibration attempt ", num_calibration_attempts, " failed!");
							phase = CALIBRATION;
						}
					} else{
						haveCalibKnowledge = false; 
						log(pat_health_port, textFileOut,  "In main.cpp phase CALIBRATION - laserOn FPGA command failed!");
						num_calibration_attempts++;
						log(pat_health_port, textFileOut,  "In main.cpp phase CALIBRATION - Calibration attempt ", num_calibration_attempts, " failed!");
						phase = CALIBRATION;
					}
					break;

				// Beacon acquisition phase, internal laser has to be off!
				case ACQUISITION:
					log(pat_health_port, textFileOut, "In main.cpp phase ACQUISITION - Beacon Acquisition Beginning. Switching off Cal Laser.");
					if(laserOff(fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer)){ //turn calibration laser off for acquistion
						if(track.runAcquisition(beacon, beaconWindow, maxBcnExposure)) // && (beacon.pixelCount > MIN_PIXELS_PER_GROUP))
						{
							// Acquisition passed!
							haveBeaconKnowledge = true;
							// Save all important parameters
							beaconExposure = camera.config->expose_us.read(); //save beacon exposure, pg-comment
							beaconGain = camera.config->gain_dB.read();
							calibGain = calibration.gainForExposure(beaconExposure);
							// display beacon and beaconWindow parameters, save image telemetry
							log(pat_health_port, textFileOut,  "In main.cpp phase ACQUISITION - Acquisition complete. ",
								"Beacon is at [", beacon.x, ",", beacon.y, ", exp = ", beaconExposure, ", valueMax = ", beacon.valueMax, ", valueSum = ", beacon.valueSum, ", pixelCount = ", beacon.pixelCount, "]", "gain = ", beaconGain);
							log(pat_health_port, textFileOut,  "In main.cpp phase ACQUISITION - Beacon Frame is at [x = ", beaconWindow.x, ", y = ", beaconWindow.y, ", w = ", beaconWindow.w, ", h = ", beaconWindow.h, "]");
							logImage(string("ACQUISITION"), camera, textFileOut, pat_health_port); 
							if(!bcnAlignment){
								// Set initial pointing in open-loop
								double setPointX_OL = (double) (CAMERA_WIDTH - beacon.x) + (double) offsets.x;
								double setPointY_OL = (double) (CAMERA_HEIGHT - beacon.y) + (double) offsets.y;
								calib.x = (int) setPointX_OL;
								calib.y = (int) setPointY_OL;
								track.controlOpenLoop(fsm, setPointX_OL, setPointY_OL);
								log(pat_health_port, textFileOut,  "In main.cpp phase ACQUISITION - Setting Calib to: [", calib.x, ",", calib.y, ", exp = ", calibExposure, "], gain = ", calibGain); //, ", smoothing: ", calibration.smoothing ",  smoothing: ", track.beaconSmoothing
							}
							camera.ignoreNextFrames(camera.queuedCount); //clear queue
							if(openLoop)
							{
								phase = OPEN_LOOP;
							}
							else
							{
								phase = CL_INIT;
							}
						}
						else
						{
							if(track.received_end_pat_cmd){
								logImage(std::string("ACQUISITION_DEBUG"), camera, textFileOut, pat_health_port); //save image telemetry
								break;
							}
							else if(track.received_end_process_cmd){
								logImage(std::string("ACQUISITION_DEBUG"), camera, textFileOut, pat_health_port); //save image telemetry
								OPERATIONAL = false;
								break; 
							}
							haveBeaconKnowledge = false; 
							num_acquisition_attempts++;
							log(pat_health_port, textFileOut,  "In main.cpp phase ACQUISITION - Beacon Acquisition attempt ", num_acquisition_attempts, " failed!");
							if(beaconWindow.w < CAMERA_HEIGHT){
								beaconWindow.w += 100;
								log(pat_health_port, textFileOut,  "In main.cpp phase ACQUISITION - Increasing window size to: ", beaconWindow.w, ", centered on ", beacon.x - CAMERA_WIDTH/2, beacon.y - CAMERA_HEIGHT/2, ", rel-to-ctr");
							}
							phase = ACQUISITION;
						}
					} else{
						haveBeaconKnowledge = false; 
						log(pat_health_port, textFileOut, "In main.cpp phase ACQUISITION - laserOff FPGA command failed!");
						num_acquisition_attempts++;
						log(pat_health_port, textFileOut,  "In main.cpp phase ACQUISITION - Beacon Acquisition attempt ", num_acquisition_attempts, " failed!");
						phase = ACQUISITION;
					}
					break;

				// Initialize closed-loop double window tracking
				case CL_INIT:
					calibWindow.x = calib.x - TRACK_ACQUISITION_WINDOW/2;
					calibWindow.y = calib.y - TRACK_ACQUISITION_WINDOW/2;
					calibWindow.w = TRACK_ACQUISITION_WINDOW;
					calibWindow.h = TRACK_ACQUISITION_WINDOW;
					log(pat_health_port, textFileOut,  "In main.cpp phase CL_INIT - ",
					"calibWindow.x = ", calibWindow.x, "calibWindow.y = ", calibWindow.y,
					"calibWindow.w = ", calibWindow.w, "calibWindow.h = ", calibWindow.h);
					// Initial values will be uncertain
					calib.pixelCount = 0;
					// Next up is beacon spot frame
					phase = CL_BEACON;
					// Save time
					beginTime = steady_clock::now();
					break;

				// Process new frame of beacon spot
				case CL_BEACON:
					if(laserOff(fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer)){ //turn calibration laser off for beacon
						camera.config->expose_us.write(beaconExposure); //set cam to beacon exposure, beaconExposure is beacon exposure, pg
						camera.setWindow(beaconWindow);
						camera.requestFrame(); //queue beacon frame, pg-comment
						if(camera.waitForFrame())
						{
							Image frame(camera, textFileOut, pat_health_port); //track.beaconSmoothing
							//save image debug telemetry
							// std::string nameTag = std::string("CL_BEACON_DEBUG");
							// std::string imageFileName = pathName + timeStamp() + std::string("_") + nameTag + std::string("_exp_") + std::to_string(camera.config->expose_us.read()) + std::string(".png");
							// log(pat_health_port, textFileOut, "In main.cpp phase CL_BEACON - Saving image telemetry as: ", imageFileName);
							// frame.savePNG(imageFileName);
							if(frame.histBrightest > TRACK_ACQUISITION_BRIGHTNESS)
							{
								cl_beacon_num_groups = frame.performPixelGrouping();
								if(cl_beacon_num_groups > 0)
								{
									spotIndex = track.findSpotCandidate(frame, beacon, &propertyDifference);
									if(spotIndex >= 0)
									{
										if(frame.groups[spotIndex].valueMax > TRACK_ACQUISITION_BRIGHTNESS)
										{
											Group& spot = frame.groups[spotIndex];
											// Check spot properties
											if(propertyDifference < TRACK_MAX_SPOT_DIFFERENCE)
											{
												haveBeaconKnowledge = true;
												// Update values if confident
												beacon.x = frame.area.x + spot.x;
												beacon.y = frame.area.y + spot.y;
												beacon.valueMax = spot.valueMax;
												beacon.valueSum = spot.valueSum;
												beacon.pixelCount = spot.pixelCount;
												track.updateTrackingWindow(frame, spot, beaconWindow);
												beaconExposure = track.controlExposure(beacon.valueMax, beaconExposure, maxBcnExposure);  //auto-tune exposure
												// If sending beacon angle errors to the bus adcs
												if(sendBusFeedback){
													check_tx_adcs = steady_clock::now(); // Record current time
													elapsed_time_tx_adcs = check_tx_adcs - time_prev_tx_adcs; // Calculate time since heartbeat tlm
													if(elapsed_time_tx_adcs > period_tx_adcs) //pg
													{
														error_angles bus_feedback = centroid2angles(beacon.x, beacon.y);
														log(pat_health_port, textFileOut, "In main.cpp phase CL_BEACON - Sending Bus Feedback Error Angles: ",
														"(X,Y) = (",bus_feedback.angle_x_radians,", ",bus_feedback.angle_y_radians,"), ",
														"from beacon centroid: (cx,cy) = (",beacon.x,", ",beacon.y,")");
														send_packet_tx_adcs(tx_packets_port, bus_feedback.angle_x_radians, bus_feedback.angle_y_radians);
														time_prev_tx_adcs = steady_clock::now(); // Record time of message		
													}
												}
											}
											else
											{
												log(pat_health_port, textFileOut,  "In main.cpp phase CL_BEACON - Rapid beacon spot property change: ",
												"(propertyDifference = ", propertyDifference, ") >= (TRACK_MAX_SPOT_DIFFERENCE = ",  TRACK_MAX_SPOT_DIFFERENCE, "). ",
												"Prev: [ x = ", beacon.x, ", y = ", beacon.y, ", pixelCount = ", beacon.pixelCount, ", valueMax = ", beacon.valueMax,
												"New: [ x = ", frame.area.x + spot.x, ", y = ", frame.area.y + spot.y, ", pixelCount = ", spot.pixelCount, ", valueMax = ", spot.valueMax);
												if(haveBeaconKnowledge) // Beacon Loss Scenario
												{
													log(pat_health_port, textFileOut,  "In main.cpp phase CL_BEACON - Beacon timeout started...");
													haveBeaconKnowledge = false;
													startBeaconLoss = steady_clock::now(); // Record time of Loss
												}
											}
										}
										else
										{
											log(pat_health_port, textFileOut, "In main.cpp phase CL_BEACON - Switching Failure: ",
											"(frame.groups[spotIndex].valueMax =  ", frame.groups[spotIndex].valueMax, ") <= (TRACK_ACQUISITION_BRIGHTNESS = ", TRACK_ACQUISITION_BRIGHTNESS,")");
											if(haveBeaconKnowledge) // Beacon Loss Scenario
											{
												log(pat_health_port, textFileOut,  "In main.cpp phase CL_BEACON - Beacon timeout started...");
												haveBeaconKnowledge = false;
												startBeaconLoss = steady_clock::now(); // Record time of Loss
											}
										}
									}
									else
									{
										log(pat_health_port, textFileOut, "In main.cpp phase CL_BEACON - Switching Failure: ",
										"(spotIndex = ", spotIndex, ") < 0");
										if(haveBeaconKnowledge) // Beacon Loss Scenario
										{
											log(pat_health_port, textFileOut,  "In main.cpp phase CL_BEACON - Beacon timeout started...");
											haveBeaconKnowledge = false;
											startBeaconLoss = steady_clock::now(); // Record time of Loss
										}
									}
								}
								else
								{
									log(pat_health_port, textFileOut, "In main.cpp phase CL_BEACON - Switching Failure: ",
									"(cl_beacon_num_groups = ", cl_beacon_num_groups, ") <= 0");
									if(haveBeaconKnowledge) // Beacon Loss Scenario
									{
										log(pat_health_port, textFileOut,  "In main.cpp phase CL_BEACON - Beacon timeout started...");
										haveBeaconKnowledge = false;
										startBeaconLoss = steady_clock::now(); // Record time of Loss
									}
								}					
							} 
							else
							{
								log(pat_health_port, textFileOut, "In main.cpp phase CL_BEACON - Switching Failure: ",
								"(frame.histBrightest = ", frame.histBrightest, ") <= (TRACK_ACQUISITION_BRIGHTNESS = ", TRACK_ACQUISITION_BRIGHTNESS,")");
								if(haveBeaconKnowledge) // Beacon Loss Scenario
								{
									log(pat_health_port, textFileOut,  "In main.cpp phase CL_BEACON - Beacon timeout started...");
									haveBeaconKnowledge = false;
									startBeaconLoss = steady_clock::now(); // Record time of Loss
								}
							}
						}
						else
						{
							log(pat_health_port, textFileOut,  "In main.cpp phase CL_BEACON - camera.waitForFrame() Failed! Camera.error: ", camera.error);
						}
					} else{
						log(pat_health_port, textFileOut, "In main.cpp phase CL_BEACON - laserOff FPGA command failed!");
						if(haveBeaconKnowledge) // Beacon Loss Scenario
						{
							log(pat_health_port, textFileOut,  "In main.cpp phase CL_BEACON - Beacon timeout started...");
							haveBeaconKnowledge = false;
							startBeaconLoss = steady_clock::now(); // Record time of Loss
						}
					}
					phase = CL_CALIB;
					if(!haveBeaconKnowledge) //Check timeout and return to acquisition if loss criterion met
					{
						time_point<steady_clock> checkBeaconLoss = steady_clock::now(); // Record current time
						duration<double> elapsedBeaconLoss = checkBeaconLoss - startBeaconLoss; // Calculate time since beacon loss
						if(elapsedBeaconLoss > waitBeaconLoss) //pg
						{
							log(pat_health_port, textFileOut,  "In main.cpp phase CL_BEACON - Beacon lost. Returning to acquisition.");
							phase = ACQUISITION; // Beacon completely lost, return to ACQUISITION, pg
							break; //pg
						}
					}
					break;

				// Process new frame of calib laser spot
				case CL_CALIB:
					if(laserOn(pat_health_port, textFileOut, fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer)){ //turn calibration laser on for calib
						camera.config->expose_us.write(calibExposure); //set cam to calib exposure, pg
						camera.setWindow(calibWindow);
						camera.requestFrame(); //queue calib frame, pg-comment
						if(camera.waitForFrame())
						{
							Image frame(camera, textFileOut, pat_health_port); //calibration.smoothing						
							// //save image debug telemetry
							// std::string nameTag = std::string("CL_CALIB_DEBUG");
							// std::string imageFileName = pathName + timeStamp() + std::string("_") + nameTag + std::string("_exp_") + std::to_string(camera.config->expose_us.read()) + std::string(".png");
							// log(pat_health_port, textFileOut, "In main.cpp phase CL_CALIB - Saving image telemetry as: ", imageFileName);
							// frame.savePNG(imageFileName);
							if(frame.histBrightest > CALIB_MIN_BRIGHTNESS)
							{
								cl_calib_num_groups = frame.performPixelGrouping();
								if(cl_calib_num_groups > 0)
								{
									spotIndex = track.findSpotCandidate(frame, calib, &propertyDifference);
									if(spotIndex >= 0)
									{
										if(frame.groups[spotIndex].valueMax > CALIB_MIN_BRIGHTNESS)
										{
											Group& spot = frame.groups[spotIndex];
											// Check spot properties
											if(propertyDifference < TRACK_MAX_SPOT_DIFFERENCE)
											{
												haveCalibKnowledge = true;
												// Update values if confident
												calib.x = frame.area.x + spot.x;
												calib.y = frame.area.y + spot.y;
												calib.valueMax = spot.valueMax;
												calib.valueSum = spot.valueSum;
												calib.pixelCount = spot.pixelCount;
												track.updateTrackingWindow(frame, spot, calibWindow);
												
												// Control in closed loop!
												double setPointX = (double) (CAMERA_WIDTH - beacon.x) + (double) offsets.x;
												double setPointY = (double) (CAMERA_HEIGHT - beacon.y) + (double) offsets.y;
												track.control(fsm, calib.x, calib.y, setPointX, setPointY);

												check_csv_write = steady_clock::now(); // Record current time
												elapsed_time_csv_write = check_csv_write - time_prev_csv_write; // Calculate time since csv write
												if(elapsed_time_csv_write > period_csv_write)
												{
													// Save for CSV
													now = steady_clock::now();
													diff = now - beginTime;
													csvData.insert(make_pair(diff.count(), CSVdata(beacon.x, beacon.y, beaconExposure,
														calib.x, calib.y, setPointX, setPointY, calibExposure)));
													//CSVdata members: double bcnX, bcnY, bcnExp, calX, calY, calSetX, calSetY, calExp;
													time_prev_csv_write = now; // Record time of csv write	
													if(!haveCsvData){haveCsvData = true;}	
												}

												if(dithering_on){
													check_dither = steady_clock::now(); // Record current time
													elapsed_time_dither = check_dither - time_prev_dither; // Calculate time since last tx offset calculation
													if(elapsed_time_dither > period_dither){
														if(dither_count == 0){
															offset_x_init = offsets.x; offset_y_init = offsets.y; //initialize reference point
														} else{
															log(pat_health_port, textFileOut, "In main.cpp - MAIN - phase ", phaseNames[phase]," - Dithering Tx offsets.");
															ditherOffsets(pat_health_port, textFileOut, offsets, dither_count, offset_x_init, offset_y_init);
														} 
														dither_count++;
														time_prev_dither = steady_clock::now();
													}
												}
											}
											else
											{
												log(pat_health_port, textFileOut,  "In main.cpp phase CL_CALIB - Rapid calib spot property change: ",
												"(propertyDifference = ", propertyDifference, ") >= (TRACK_MAX_SPOT_DIFFERENCE = ",  TRACK_MAX_SPOT_DIFFERENCE, "). ",
												"Prev: [ x = ", calib.x, ", y = ", calib.y, ", pixelCount = ", calib.pixelCount, ", valueMax = ", calib.valueMax,
												"New: [ x = ", frame.area.x + spot.x, ", y = ", frame.area.y + spot.y, ", pixelCount = ", spot.pixelCount, ", valueMax = ", spot.valueMax);
											}
										}
										else
										{
											log(pat_health_port, textFileOut, "In main.cpp phase CL_CALIB - Switching Failure: ",
											"(frame.groups[spotIndex].valueMax = ", frame.groups[spotIndex].valueMax, ") <= (CALIB_MIN_BRIGHTNESS/4 = ", CALIB_MIN_BRIGHTNESS/4,")");							
											if(haveCalibKnowledge)
											{						
												log(pat_health_port, textFileOut,  "In main.cpp phase CL_CALIB - Calib spot vanished!");
												haveCalibKnowledge = false;
												startCalibLoss = steady_clock::now(); // Record time of Loss
											}
										}
									}
									else
									{
										log(pat_health_port, textFileOut, "In main.cpp phase CL_CALIB - Switching Failure: ",
										"(spotIndex = ", spotIndex, ") < 0");						
										if(haveCalibKnowledge)
										{						
											log(pat_health_port, textFileOut,  "In main.cpp phase CL_CALIB - Calib spot vanished!");
											haveCalibKnowledge = false;
											startCalibLoss = steady_clock::now(); // Record time of Loss
										}
									}
								}	
								else
								{
									log(pat_health_port, textFileOut, "In main.cpp phase CL_CALIB - Switching Failure: ",
									"(cl_calib_num_groups = ", cl_calib_num_groups, ") <= 0");						
									if(haveCalibKnowledge)
									{						
										log(pat_health_port, textFileOut,  "In main.cpp phase CL_CALIB - Calib spot vanished!");
										haveCalibKnowledge = false;
										startCalibLoss = steady_clock::now(); // Record time of Loss
									}
								}			
							}
							else
							{
								log(pat_health_port, textFileOut, "In main.cpp phase CL_CALIB - Switching Failure: ",
								"(frame.histBrightest = ", frame.histBrightest, ") <= (CALIB_MIN_BRIGHTNESS = ", CALIB_MIN_BRIGHTNESS,")");						
								if(haveCalibKnowledge)
								{						
									log(pat_health_port, textFileOut,  "In main.cpp phase CL_CALIB - Calib spot vanished!");
									haveCalibKnowledge = false;
									startCalibLoss = steady_clock::now(); // Record time of Loss
								}
							}
						}
						else
						{
							log(pat_health_port, textFileOut,  "In main.cpp phase CL_CALIB - camera.waitForFrame() Failed! Camera.error: ", camera.error);
							if(haveCalibKnowledge)
							{						
								log(pat_health_port, textFileOut,  "In main.cpp phase CL_CALIB - Calib spot vanished!");
								haveCalibKnowledge = false;
								startCalibLoss = steady_clock::now(); // Record time of Loss
							}
						}
					} else{
						log(pat_health_port, textFileOut, "In main.cpp phase CL_CALIB - laserOn FPGA command failed!");
						if(haveCalibKnowledge)
						{						
							log(pat_health_port, textFileOut,  "In main.cpp phase CL_CALIB - Calib spot vanished!");
							haveCalibKnowledge = false;
							startCalibLoss = steady_clock::now(); // Record time of Loss
						}
					}
					phase = CL_BEACON;
					if(!haveCalibKnowledge) //Check timeout and return to acquisition if loss criterion met
					{
						time_point<steady_clock> checkCalibLoss = steady_clock::now(); // Record current time
						duration<double> elapsedCalibLoss = checkCalibLoss - startCalibLoss; // Calculate time since calib loss
						if(elapsedCalibLoss > waitCalibLoss) 
						{
							log(pat_health_port, textFileOut,  "In main.cpp phase CL_CALIB - Calib loss timeout. Switching to open loop.");
							phase = OPEN_LOOP;
							break; 
						}
					}
					break;

				// Control in open-loop, sampling only beacon spot!
				case OPEN_LOOP:
					if(laserOff(fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer)){ //turn calibration laser off for open-loop
						// Request new frame
						camera.config->expose_us.write(beaconExposure); //set cam to beacon exposure, beaconExposure is beacon exposure, pg
						camera.setWindow(beaconWindow);
						camera.requestFrame(); //queue beacon frame, pg-comment
						if(camera.waitForFrame())
						{
							Image frame(camera, textFileOut, pat_health_port); //, track.beaconSmoothing
							// //save image debug telemetry
							// std::string nameTag = std::string("OPEN_LOOP_DEBUG");
							// std::string imageFileName = pathName + timeStamp() + std::string("_") + nameTag + std::string("_exp_") + std::to_string(camera.config->expose_us.read()) + std::string(".png");
							// log(pat_health_port, textFileOut, "In main.cpp phase OPEN_LOOP - Saving image telemetry as: ", imageFileName);
							// frame.savePNG(imageFileName);
							if(frame.histBrightest > TRACK_ACQUISITION_BRIGHTNESS)
							{
								cl_beacon_num_groups = frame.performPixelGrouping();
								if(cl_beacon_num_groups > 0)
								{
									spotIndex = track.findSpotCandidate(frame, beacon, &propertyDifference);
									if(spotIndex >= 0)
									{
										if(frame.groups[spotIndex].valueMax > TRACK_ACQUISITION_BRIGHTNESS)
										{
											Group& spot = frame.groups[spotIndex];
											// Check spot properties
											if(propertyDifference < TRACK_MAX_SPOT_DIFFERENCE)
											{
												haveBeaconKnowledge = true;
												// Update values if confident
												beacon.x = frame.area.x + spot.x;
												beacon.y = frame.area.y + spot.y;
												beacon.valueMax = spot.valueMax;
												beacon.valueSum = spot.valueSum;
												beacon.pixelCount = spot.pixelCount;
												track.updateTrackingWindow(frame, spot, beaconWindow);
												beaconExposure = track.controlExposure(beacon.valueMax, beaconExposure, maxBcnExposure);  //auto-tune exposure
												if(!bcnAlignment){
													// Control pointing in open-loop
													double setPointX_OL = (double) (CAMERA_WIDTH - beacon.x) + (double) offsets.x;
													double setPointY_OL = (double) (CAMERA_HEIGHT - beacon.y) + (double) offsets.y;
													calib.x = (int) setPointX_OL;
													calib.y = (int) setPointY_OL;
													track.controlOpenLoop(fsm, setPointX_OL, setPointY_OL);
												}
												check_csv_write = steady_clock::now(); // Record current time
												elapsed_time_csv_write = check_csv_write - time_prev_csv_write; // Calculate time since csv write
												if(elapsed_time_csv_write > period_csv_write)
												{
													// Save for CSV
													now = steady_clock::now();
													diff = now - beginTime;
													csvData.insert(make_pair(diff.count(), CSVdata(beacon.x, beacon.y, beaconExposure,
														calib.x, calib.y, beacon.x, beacon.y, calibExposure)));
													//CSVdata members: double bcnX, bcnY, bcnExp, calX, calY, calSetX, calSetY, calExp;
													time_prev_csv_write = now; // Record time of csv write
													if(!haveCsvData){haveCsvData = true;}			
												}
												// If sending beacon angle errors to the bus adcs
												//standard sampling frequency is about 1/(40ms) = 25Hz, reduced 25x to 1Hz (TBR - Sychronization)
												if(sendBusFeedback){
													check_tx_adcs = steady_clock::now(); // Record current time
													elapsed_time_tx_adcs = check_tx_adcs - time_prev_tx_adcs; // Calculate time since heartbeat tlm
													if(elapsed_time_tx_adcs > period_tx_adcs) //pg
													{
														error_angles bus_feedback = centroid2angles(beacon.x, beacon.y);
														log(pat_health_port, textFileOut, "In main.cpp phase CL_BEACON - Sending Bus Feedback Error Angles: ",
														"(X,Y) = (",bus_feedback.angle_x_radians,", ",bus_feedback.angle_y_radians,"), ",
														"from beacon centroid: (cx,cy) = (",beacon.x,", ",beacon.y,")");
														send_packet_tx_adcs(tx_packets_port, bus_feedback.angle_x_radians, bus_feedback.angle_y_radians);
														time_prev_tx_adcs = steady_clock::now(); // Record time of message		
													}
												}
											}
											else
											{
												log(pat_health_port, textFileOut,  "In main.cpp phase OPEN_LOOP - Rapid beacon spot property change: ",
												"(propertyDifference = ", propertyDifference, ") >= (TRACK_MAX_SPOT_DIFFERENCE = ",  TRACK_MAX_SPOT_DIFFERENCE, "). ",
												"Prev: [ x = ", beacon.x, ", y = ", beacon.y, ", pixelCount = ", beacon.pixelCount, ", valueMax = ", beacon.valueMax,
												"New: [ x = ", frame.area.x + spot.x, ", y = ", frame.area.y + spot.y, ", pixelCount = ", spot.pixelCount, ", valueMax = ", spot.valueMax);
												if(haveBeaconKnowledge) // Beacon Loss Scenario
												{
													log(pat_health_port, textFileOut,  "In main.cpp phase OPEN_LOOP - Beacon timeout started...");
													haveBeaconKnowledge = false;
													startBeaconLoss = steady_clock::now(); // Record time of Loss
												}
											}
										}
										else
										{
											log(pat_health_port, textFileOut, "In main.cpp phase OPEN_LOOP - Switching Failure: ",
											"(frame.groups[spotIndex].valueMax =  ", frame.groups[spotIndex].valueMax, ") <= (TRACK_ACQUISITION_BRIGHTNESS = ", TRACK_ACQUISITION_BRIGHTNESS,")");
											if(haveBeaconKnowledge) // Beacon Loss Scenario
											{
												log(pat_health_port, textFileOut,  "In main.cpp phase OPEN_LOOP - Beacon timeout started...");
												haveBeaconKnowledge = false;
												startBeaconLoss = steady_clock::now(); // Record time of Loss
											}
										}
									}
									else
									{
										log(pat_health_port, textFileOut, "In main.cpp phase OPEN_LOOP - Switching Failure: ",
										"(spotIndex = ", spotIndex, ") < 0");
										if(haveBeaconKnowledge) // Beacon Loss Scenario
										{
											log(pat_health_port, textFileOut,  "In main.cpp phase OPEN_LOOP - Beacon timeout started...");
											haveBeaconKnowledge = false;
											startBeaconLoss = steady_clock::now(); // Record time of Loss
										}
									}
								}
								else
								{
									log(pat_health_port, textFileOut, "In main.cpp phase OPEN_LOOP - Switching Failure: ",
									"(cl_beacon_num_groups = ", cl_beacon_num_groups, ") <= 0");
									if(haveBeaconKnowledge) // Beacon Loss Scenario
									{
										log(pat_health_port, textFileOut,  "In main.cpp phase OPEN_LOOP - Beacon timeout started...");
										haveBeaconKnowledge = false;
										startBeaconLoss = steady_clock::now(); // Record time of Loss
									}
								}					
							} 
							else
							{
								log(pat_health_port, textFileOut, "In main.cpp phase OPEN_LOOP - Switching Failure: ",
								"(frame.histBrightest = ", frame.histBrightest, ") <= (TRACK_ACQUISITION_BRIGHTNESS = ", TRACK_ACQUISITION_BRIGHTNESS,")");
								if(haveBeaconKnowledge) // Beacon Loss Scenario
								{
									log(pat_health_port, textFileOut,  "In main.cpp phase OPEN_LOOP - Beacon timeout started...");
									haveBeaconKnowledge = false;
									startBeaconLoss = steady_clock::now(); // Record time of Loss
								}
							}
						}
						else
						{
							log(pat_health_port, textFileOut,  "In main.cpp phase OPEN_LOOP - camera.waitForFrame() Failed! Error: ", camera.error);
							if(haveBeaconKnowledge) // Beacon Loss Scenario
							{
								log(pat_health_port, textFileOut,  "In main.cpp phase OPEN_LOOP - Beacon timeout started...");
								haveBeaconKnowledge = false;
								startBeaconLoss = steady_clock::now(); // Record time of Loss
							}
						}
					} else{
						log(pat_health_port, textFileOut, "In main.cpp phase OPEN_LOOP - laserOff FPGA command failed!");
						if(haveBeaconKnowledge) // Beacon Loss Scenario
						{
							log(pat_health_port, textFileOut,  "In main.cpp phase OPEN_LOOP - Beacon timeout started...");
							haveBeaconKnowledge = false;
							startBeaconLoss = steady_clock::now(); // Record time of Loss
						}
					}			
					
					if(!haveBeaconKnowledge) //Check timeout and return to acquisition if loss criterion met
					{
						time_point<steady_clock> checkBeaconLoss = steady_clock::now(); // Record current time
						duration<double> elapsedBeaconLoss = checkBeaconLoss - startBeaconLoss; // Calculate time since beacon loss
						if(elapsedBeaconLoss > waitBeaconLoss) //pg
						{
							log(pat_health_port, textFileOut,  "In main.cpp phase OPEN_LOOP - Beacon lost. Returning to acquisition.");
							phase = ACQUISITION; // Beacon completely lost, return to ACQUISITION, pg
							break; //pg
						}
					}
					break;
					
				// Graceful failure mode for when beacon is not detected: command the FSM to point the laser straight & rely on bus pointing.
				case STATIC_POINT:
					if(!static_pointing_initialized){
						// Command FSM
						if(haveCalibKnowledge){
							// Control pointing in open-loop
							double setPointX_OL = (double) CAMERA_WIDTH/2 + (double) offsets.x;
							double setPointY_OL = (double) CAMERA_HEIGHT/2 + (double) offsets.y;
							calib.x = (int) setPointX_OL;
							calib.y = (int) setPointY_OL;
							track.controlOpenLoop(fsm, setPointX_OL, setPointY_OL);
							log(pat_health_port, textFileOut,  "In main.cpp phase STATIC_POINT - Have calibration knowledge. Setting FSM to: ",
							"x_pixels = ", calib.x, ", y_pixels = ", calib.y);
						} else{
							log(pat_health_port, textFileOut,  "In main.cpp phase STATIC_POINT - Do not have calibration knowledge. Setting FSM to (x_normalized, y_normalized) = (0,0).");
							fsm.setNormalizedAngles(0,0); 
						}

						// Save for CSV
						now = steady_clock::now();
						diff = now - beginTime;
						csvData.insert(make_pair(diff.count(), CSVdata(0, 0, 0,	calib.x, calib.y, 0, 0, 0)));
						//CSVdata members: double bcnX, bcnY, bcnExp, calX, calY, calSetX, calSetY, calExp;

						static_pointing_initialized = true; 
					}
					break;

				// Fail-safe
				default:
					log(pat_health_port, textFileOut,  "In main.cpp phase default - Unknown phase encountered in switch structure. Transitioning to STATIC_POINT.");
					phase = STATIC_POINT;
					break;
			}
		} 
		// END of main PAT loop
		STANDBY = true;	
	} 
	// END of primary process loop: Standby + Main
	
	//turn off laser before ending PAT process
	log(pat_health_port, textFileOut,  "In main.cpp - Shutting off calib laser and resetting FSM to (0,0).");
	if(!laserOff(fpga_map_request_port, fpga_map_answer_port, poll_fpga_answer)){ //turn calibration laser off
		log(pat_health_port, textFileOut,  "In main.cpp - End of PAT process laserOff FPGA command failed!");
	}
	//reset FSM before ending PAT process
	fsm.resetFSM();

	if(haveCsvData){
		log(pat_health_port, textFileOut,  "In main.cpp - Saving csv file.");
		ofstream out(dataFileName);
		for(const auto& x : csvData)
		{
			out << x.first << "," << x.second.bcnX << "," << x.second.bcnY << "," << x.second.bcnExp << ","
			<< x.second.calX << "," << x.second.calY << "," << x.second.calSetX << "," << x.second.calSetY << "," << x.second.calExp << endl;
			//x.first = time since beginTime
			//	CSVdata members: double bcnX, bcnY, bcnExp, calX, calY, calSetX, calSetY, calExp;
		}
		out.close(); //close data file
		//std::cout << "CSV File Saved." << std::endl;
	}
	
	log(pat_health_port, textFileOut,  "In main.cpp - Saving text file and ending process.");
	textFileOut.close(); //close telemetry text file
    //std::cout << "TXT File Saved." << std::endl;
	pat_status_port.close();
	pat_health_port.close();
	pat_control_port.close();
	fpga_map_request_port.close();
	fpga_map_answer_port.close();
	tx_packets_port.close();
	//std::cout << "Ports Closed." << std::endl;
    context.close();
	//std::cout << "Context Closed." << std::endl;
    return 0;
}
