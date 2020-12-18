#include <unistd.h>
#include <atomic>
#include <csignal>
#include <chrono>
#include <thread>
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
//#define OFFSET_X 36 //user input from calibration
//#define OFFSET_Y 71 //user input from calibration
#define CENTROID2ANGLE_SLOPE_X 1 //user input from calibration
#define CENTROID2ANGLE_BIAS_X 0 //user input from calibration
#define CENTROID2ANGLE_SLOPE_Y 1 //user input from calibration
#define CENTROID2ANGLE_BIAS_Y 0 //user input from calibration

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
void laserOn(zmq::socket_t& fpga_map_request_port, uint8_t request_number){
	send_packet_fpga_map_request(fpga_map_request_port, (uint16_t) CALIB_CH, (uint8_t) CALIB_ON, (bool) WRITE, request_number);
	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
}

//Turn Off Calibration Laser
void laserOff(zmq::socket_t& fpga_map_request_port, uint8_t request_number){
	send_packet_fpga_map_request(fpga_map_request_port, (uint16_t) CALIB_CH, (uint8_t) CALIB_OFF, (bool) WRITE, request_number);
	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
}

//Convert Beacon Centroid to Error Angles for the Bus
struct error_angles{
	float angle_x_radians;
	float angle_y_radians;
};
error_angles centroid2angles(double centroid_x, double centroid_y){
	error_angles angles = error_angles();
	angles.angle_x_radians = (float) CENTROID2ANGLE_SLOPE_X*centroid_x + CENTROID2ANGLE_BIAS_X;
	angles.angle_y_radians = (float) CENTROID2ANGLE_SLOPE_Y*centroid_y + CENTROID2ANGLE_BIAS_Y;
	return angles;
}

atomic<bool> stop(false); //not for flight

//-----------------------------------------------------------------------------
int main() //int argc, char** argv
//-----------------------------------------------------------------------------
{	
	// https://ogbe.net/blog/zmq_helloworld.html
	// define ports for PUB/SUB (this process binds)
	std::string PAT_HEALTH_PORT = "tcp://localhost:5559"; //PUB to Housekeeping
    std::string PAT_CONTROL_PORT = "tcp://localhost:5560"; //SUB to Command Handler
    std::string FPGA_MAP_REQUEST_PORT = "tcp://localhost:5558"; //PUB to FPGA Driver
    std::string FPGA_MAP_ANSWER_PORT = "tcp://localhost:5557"; //SUB to FPGA Driver
    std::string TX_PACKETS_PORT = "tcp://localhost:5561"; //PUB to Packetizer & Bus Interface
    //std::string RX_PAT_PACKETS_PORT = "tcp://localhost:5562";  //SUB to Packetizer & Bus Interface 

    // initialize the zmq context with 1 IO threads 
    zmq::context_t context{1}; 
    
    // Create the PUB/SUB Sockets: 

	// create the PAT_HEALTH_PORT PUB socket
	zmq::socket_t pat_health_port(context, ZMQ_PUB); 
    pat_health_port.connect(PAT_HEALTH_PORT); // connect to the transport bind(PAT_HEALTH_PORT)
    
    // create the PAT_CONTROL_PORT SUB socket
    zmq::socket_t pat_control_port(context, ZMQ_SUB); 
    pat_control_port.connect(PAT_CONTROL_PORT); // connect to the transport
    pat_control_port.set(zmq::sockopt::subscribe, ""); // set the socket options such that we receive all messages. we can set filters here. this "filter" ("" and 0) subscribes to all messages.	
    
    // create the FPGA_MAP_REQUEST_PORT PUB socket
    zmq::socket_t fpga_map_request_port(context, ZMQ_PUB); 
    fpga_map_request_port.connect(FPGA_MAP_REQUEST_PORT); // connect to the transport
    
    // create the FPGA_MAP_ANSWER_PORT SUB socket
    zmq::socket_t fpga_map_answer_port(context, ZMQ_SUB); // create the FPGA_MAP_ANSWER_PORT SUB socket
    fpga_map_answer_port.connect(FPGA_MAP_ANSWER_PORT); // connect to the transport
    fpga_map_answer_port.set(zmq::sockopt::subscribe, ""); // set the socket options such that we receive all messages. we can set filters here. this "filter" ("" and 0) subscribes to all messages.	

    // create the TX_PACKETS_PORT PUB socket
    zmq::socket_t tx_packets_port(context, ZMQ_PUB); 
    tx_packets_port.connect(TX_PACKETS_PORT); // connect to the transport
    
    /*
    // create the RX_PAT_PACKETS_PORT SUB socket
    zmq::socket_t rx_pat_packets_port(context, ZMQ_SUB); 
    rx_pat_packets_port.connect(RX_PAT_PACKETS_PORT); // connect to the transport
    rx_pat_packets_port.set(zmq::sockopt::subscribe, ""); // set the socket options such that we receive all messages. we can set filters here. this "filter" ("" and 0) subscribes to all messages.	
	*/
	
	// to use zmq_poll correctly, we construct this vector of pollitems (based on: https://ogbe.net/blog/zmq_helloworld.html)
	std::vector<zmq::pollitem_t> p = {{pat_control_port, 0, ZMQ_POLLIN, 0}};
	
	//Allow sockets some time (otherwise you get dropped packets)
	std::this_thread::sleep_for(std::chrono::seconds(1));
		
	//telemetry file names
	std::string textFileName = timeStamp() + string("_pat_logs.txt"); //used for text telemetry
	std::string dataFileName = timeStamp() + string("_pat_data.csv"); //used by csv data file generation

	//Generate text telemetry file, pg
	ofstream textFileOut; //stream for text telemetry
	textFileOut.open(textFileName, ios::app); //create text file and open for writing

	// Synchronization
	enum Phase { START, CALIBRATION, ACQUISITION, STATIC_POINT, OPEN_LOOP, CL_INIT, CL_BEACON, CL_CALIB };
	const char *phaseNames[8] = {"START","CALIBRATION","ACQUISITION","STATIC_POINT","OPEN_LOOP","CL_INIT","CL_BEACON","CL_CALIB"};
	
	// Initialize execution variables
	Phase phase = START;
	Group beacon, calib;
	AOI beaconWindow, calibWindow;
	int beaconExposure = 0, spotIndex = 0, calibExposure = 0;
	int beaconGain = 0, calibGain = 0;
	bool haveBeaconKnowledge = false, haveCalibKnowledge = false;
	double propertyDifference = 0;
	//int centerOffsetX = OFFSET_X; 
	//int centerOffsetY = OFFSET_Y; 
	bool openLoop = false, staticPoint = false, sendBusFeedback = false;
	uint16_t command; 
	int command_exposure;  
	int cl_beacon_num_groups, cl_calib_num_groups;
	
	// Hardware init				
	Camera camera(textFileOut, pat_health_port);	
	//Catch camera initialization failure state in a re-initialization loop:
	bool camera_initialized = camera.initialize();	
	while(!camera_initialized){
		log(pat_health_port, textFileOut, "In main.cpp - Camera Initialization Failed! Error:", camera.error);
		// Listen for exit command 		
		zmq::poll(p.data(), 1, 1000); // when timeout_ms (the third argument here) is -1, then block until ready to receive (based on: https://ogbe.net/blog/zmq_helloworld.html)
		if(p[0].revents & ZMQ_POLLIN){
			// received something on the first (only) socket
			command = receive_packet_pat_control(pat_control_port);
			if(command == CMD_END_PAT){
				log(pat_health_port, textFileOut, "In main.cpp - Received CMD_END_PAT command. Exiting...");
				exit(-1);
			}
		}
		//Try to initialize again
		camera_initialized = camera.initialize();
	}	
	log(pat_health_port, textFileOut, "In main.cpp Camera Connection Initialized");
	
	FSM fsm(textFileOut, pat_health_port, fpga_map_request_port);
	Calibration calibration(camera, fsm, textFileOut, pat_health_port);
	Tracking track(camera, calibration, textFileOut, pat_health_port);
	
	// Standby for command:
	bool STANDBY = true;
	while(STANDBY){
		log(pat_health_port, textFileOut, "In main.cpp - Standing by for command..."); //health heartbeat		
		// Listen for command 		
		zmq::poll(p.data(), 1, 1000); // when timeout_ms (the third argument here) is -1, then block until ready to receive (based on: https://ogbe.net/blog/zmq_helloworld.html)
		if(p[0].revents & ZMQ_POLLIN)
		{
			// received something on the first (only) socket
			char command_data[CMD_PAYLOAD_SIZE];
			command = receive_packet_pat_control(pat_control_port, command_data);
			switch(command)
			{
				case CMD_START_PAT:
					log(pat_health_port, textFileOut, "In main.cpp - Received CMD_START_PAT command. Proceeding to main PAT loop...");
					STANDBY = false;
					break;
					
				case CMD_END_PAT:
					log(pat_health_port, textFileOut, "In main.cpp - Received CMD_END_PAT command. Exiting...");
					exit(-1);
					
				case CMD_START_PAT_OPEN_LOOP:
					log(pat_health_port, textFileOut, "In main.cpp - Received CMD_START_PAT_OPEN_LOOP command. Proceeding to main PAT loop...");
					openLoop = true;
					STANDBY = false;
					break;
				
				case CMD_START_PAT_STATIC_POINT:
					log(pat_health_port, textFileOut, "In main.cpp - Received CMD_START_PAT_STATIC_POINT command. Proceeding to main PAT loop...");
					staticPoint = true;
					STANDBY = false;
					break;
					
				case CMD_START_PAT_BUS_FEEDBACK:
					log(pat_health_port, textFileOut, "In main.cpp - Received CMD_START_PAT_BUS_FEEDBACK command. Proceeding to main PAT loop...");
					sendBusFeedback = true;
					STANDBY = false;
					break;		
					
				case CMD_GET_IMAGE:
					//set to commanded exposure
					command_exposure = atoi(command_data); 
					if(command_exposure < MIN_EXPOSURE){command_exposure = MIN_EXPOSURE;}
					if(command_exposure > MAX_EXPOSURE){command_exposure = MAX_EXPOSURE;}
					log(pat_health_port, textFileOut, "In main.cpp - Received CMD_GET_IMAGE command with exposure = ", command_exposure);
					camera.config->expose_us.write(command_exposure);

					//set to sufficiently large window size (but not too large)
					camera.setCenteredWindow(CAMERA_WIDTH/2, CAMERA_HEIGHT/2, CAMERA_HEIGHT);

					//save image
					logImage(string("CMD_GET_IMAGE"), camera, textFileOut, pat_health_port); 
					break;		
							
				case CMD_CALIB_TEST:
					log(pat_health_port, textFileOut, "In main.cpp - Received CMD_CALIB_TEST command.");
					//Run calibration:
					laserOn(fpga_map_request_port, 0); //TBR request number argument, turn calibration laser on	
					if(calibration.run(calib)) //sets calib exposure
					{
						calibExposure = camera.config->expose_us.read(); //save calib exposure
						log(pat_health_port, textFileOut, "In main.cpp CMD_CALIB_TEST - Calibration complete. Calib Exposure = ", calibExposure, " us.");
					}
					else
					{
						log(pat_health_port, textFileOut,  "In main.cpp CMD_CALIB_TEST - Calibration failed!");
					}	
					laserOff(fpga_map_request_port, 0); //TBR request number argument, turn calibration laser off				
					break;
				
				case CMD_CALIB_LASER_TEST:
					//set to commanded exposure
					command_exposure = atoi(command_data); 
					if(command_exposure < MIN_EXPOSURE){command_exposure = MIN_EXPOSURE;}
					if(command_exposure > MAX_EXPOSURE){command_exposure = MAX_EXPOSURE;}
					log(pat_health_port, textFileOut, "In main.cpp - Received CMD_CALIB_LASER_TEST command with exposure = ", command_exposure);
					camera.config->expose_us.write(command_exposure);		

					//set to sufficiently large window size (but not too large)
					camera.setCenteredWindow(CAMERA_WIDTH/2, CAMERA_HEIGHT/2, CALIB_BIG_WINDOW);

					//switch laser on and ensure FSM is centered
					laserOn(fpga_map_request_port, 0);
					fsm.setNormalizedAngles(0,0); 	

					//save image
					logImage(string("CMD_CALIB_LASER_TEST"), camera, textFileOut, pat_health_port);

					//switch laser off
					laserOff(fpga_map_request_port, 0); //TBR request number argument, turn calibration laser off
					break;

				case CMD_FSM_TEST:
					//set to commanded exposure
					command_exposure = atoi(command_data); 
					if(command_exposure < MIN_EXPOSURE){command_exposure = MIN_EXPOSURE;}
					if(command_exposure > MAX_EXPOSURE){command_exposure = MAX_EXPOSURE;}
					log(pat_health_port, textFileOut, "In main.cpp - Received CMD_FSM_TEST command with exposure = ", command_exposure);
					camera.config->expose_us.write(command_exposure);		

					//set to sufficiently large window size (but not too large)
					camera.setCenteredWindow(CAMERA_WIDTH/2, CAMERA_HEIGHT/2, CALIB_BIG_WINDOW);	

					//switch laser on
					laserOn(fpga_map_request_port, 0);

					//save images at various FSM settings
					fsm.setNormalizedAngles(0,0);
					std::this_thread::sleep_for(std::chrono::milliseconds(3));
					logImage(string("CMD_FSM_TEST_Center"), camera, textFileOut, pat_health_port); 
					std::this_thread::sleep_for(std::chrono::milliseconds(1000));

					fsm.setNormalizedAngles(1,0);
					std::this_thread::sleep_for(std::chrono::milliseconds(3));
					logImage(string("CMD_FSM_TEST_X"), camera, textFileOut, pat_health_port);
					std::this_thread::sleep_for(std::chrono::milliseconds(1000));

					fsm.setNormalizedAngles(0,1);
					std::this_thread::sleep_for(std::chrono::milliseconds(3));
					logImage(string("CMD_FSM_TEST_Y"), camera, textFileOut, pat_health_port);  
					std::this_thread::sleep_for(std::chrono::milliseconds(1000));

					fsm.setNormalizedAngles(1,1);
					std::this_thread::sleep_for(std::chrono::milliseconds(3));
					logImage(string("CMD_FSM_TEST_XY"), camera, textFileOut, pat_health_port); 
					std::this_thread::sleep_for(std::chrono::milliseconds(1000));

					//switch laser off
					laserOff(fpga_map_request_port, 0); //TBR request number argument, turn calibration laser off
					break;
					
				default:
					log(pat_health_port, textFileOut, "In main.cpp - Received unknown command: ", command);
			}	
		}
	}		
	// END of STANDBY loop
	
	// CSV data saving for main PAT loop
	time_point<steady_clock> beginTime;
	map<double, CSVdata> csvData;

	//Beacon Loss Timing, pg
	time_point<steady_clock> startBeaconLoss; //pg
	duration<double> waitBeaconLoss(5.0); //wait time before switching back to ACQUISITION, pg
	
	// Killing app handler (Enables graceful Ctrl+C exit - not for flight)
	signal(SIGINT, [](int signum) { stop = true; });
		
	// Main PAT Loop
	for(int i = 1; ; i++)
	{	
		// Allow graceful exit with Ctrl-C (not for flight)
		if(stop) break;
		
		// Listen for CMD_END_PAT 		
		zmq::poll(p.data(), 1, 10); // when timeout_ms (the third argument here) is -1, then block until ready to receive (based on: https://ogbe.net/blog/zmq_helloworld.html)
		if(p[0].revents & ZMQ_POLLIN) {
			// received something on the first (only) socket
			command = receive_packet_pat_control(pat_control_port);
			if (command == CMD_END_PAT){
			  break;
			}
		}
		
		// Periodic health update
		if(true) //i % 100 == 0, standard sampling frequency is about 1/(40ms) = 25Hz, reduced 100x to 0.25Hz (one every 4 seconds < 5 second housekeeping aliveness check)
		{
			if(phase == OPEN_LOOP){
				if(haveBeaconKnowledge){
					log(pat_health_port, textFileOut, "In main.cpp phase ", phaseNames[phase]," - Beacon is at [", beacon.x, ",", beacon.y, "]");
				} else{
					log(pat_health_port, textFileOut, "In main.cpp phase ", phaseNames[phase]," - No idea where beacon is.");
				}
			} else{
				if(beaconExposure == TRACK_MIN_EXPOSURE) log(pat_health_port, textFileOut,  "In main.cpp console update - Minimum beacon exposure reached!"); //notification when exposure limits reached, pg
				if(beaconExposure == TRACK_MAX_EXPOSURE) log(pat_health_port, textFileOut,  "In main.cpp console update - Maximum beacon exposure reached!");
				if(haveBeaconKnowledge){
					log(pat_health_port, textFileOut, "In main.cpp phase ", phaseNames[phase]," - Beacon is at [", beacon.x, ",", beacon.y, ", exp = ", beaconExposure, "]");
				} else{
					log(pat_health_port, textFileOut, "In main.cpp phase ", phaseNames[phase]," - No idea where beacon is.");
				}
				if(haveCalibKnowledge){
					log(pat_health_port, textFileOut, "In main.cpp phase ", phaseNames[phase]," - Calib is at [", calib.x, ",", calib.y, ", exp = ", calibExposure, "]");
				} else{
					log(pat_health_port, textFileOut, "In main.cpp phase ", phaseNames[phase]," - No idea where calib is.");
				}
			}			
			i = 0;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));					
		//PAT Phases:		
		switch(phase)
		{
			case START:
				log(pat_health_port, textFileOut, "In main.cpp phase START - Calibration Starting...");
				phase = CALIBRATION;
				break;

			// Calibration phase, internal laser has to be turned on, ran just once
			case CALIBRATION:
				laserOn(fpga_map_request_port, 0); //TBR request number argument, turn calibration laser on
				if(calibration.run(calib)) //sets calib exposure, pg-comment
				{
					calibExposure = camera.config->expose_us.read(); //save calib exposure, pg
					log(pat_health_port, textFileOut, "In main.cpp phase CALIBRATION - Calibration complete. Calib Exposure = ", calibExposure, " us.");
					if(staticPoint)
					{
						phase = STATIC_POINT;
					}
					else
					{
						phase = ACQUISITION;
					}
				}
				else
				{
					log(pat_health_port, textFileOut,  "In main.cpp phase CALIBRATION - Calibration failed!");
					phase = START;
				}
				break;

			// Beacon acquisition phase, internal laser has to be off!
			case ACQUISITION:
				log(pat_health_port, textFileOut, "In main.cpp phase ACQUISITION - Beacon Acquisition Beginning. Switch off Cal Laser.");
				laserOff(fpga_map_request_port, 0); //TBR request number argument, turn calibration laser off for acquistion
				if(track.runAcquisition(beacon) && (beacon.pixelCount > MIN_PIXELS_PER_GROUP))
				{
					// Acquisition passed!
					haveBeaconKnowledge = true;
					// Save all important parameters
					beaconExposure = camera.config->expose_us.read(); //save beacon exposure, pg-comment
					beaconGain = camera.config->gain_dB.read();
					calibGain = calibration.gainForExposure(beaconExposure);
					// Set initial pointing in open-loop
					calib.x = 2*((CAMERA_WIDTH/2) + calibration.centerOffsetX) - beacon.x;
					calib.y = 2*((CAMERA_HEIGHT/2) + calibration.centerOffsetY) - beacon.y;
					log(pat_health_port, textFileOut,  "In main.cpp phase ACQUISITION - Acquisition complete. ",
						"Beacon is at [", beacon.x, ",", beacon.y, ", exp = ", beaconExposure, "] ", beaconGain, "dB smoothing", track.beaconSmoothing, 
						". Setting Calib to: [", calib.x, ",", calib.y, ", exp = ", calibExposure, "] ", calibGain, "dB smoothing", calibration.smoothing);
					logImage(string("ACQUISITION"), camera, textFileOut, pat_health_port); 
					track.controlOpenLoop(fsm, calib.x, calib.y);
					if(openLoop)
					{
						camera.requestFrame(); //queue frame, pg-comment
						phase = OPEN_LOOP;
					}
					else
					{
						phase = CL_INIT;
					}
				}
				else
				{
					log(pat_health_port, textFileOut, "In main.cpp phase ACQUISITION - Beacon Acquisition Failed! Transitioning to Static Pointing Mode...");
					phase = STATIC_POINT; 
				}
				break;

			// Initialize closed-loop double window tracking
			case CL_INIT:
				camera.ignoreNextFrames(camera.queuedCount); //clear queue
				/*
				// Init flipping windows - first window
				//camera.config->gain_dB.write(beaconGain);
				camera.config->expose_us.write(beaconExposure); //set cam to beacon exposure, beaconExposure is beacon exposure, pg
				camera.setWindow(beaconWindow);
				camera.requestFrame(); //queue beacon frame, pg-comment
				// Request second window
				//camera.config->gain_dB.write(calibGain);
				camera.config->expose_us.write(calibExposure); //set cam to calib exposure, pg
				calibWindow.x = calib.x - TRACK_ACQUISITION_WINDOW/2;
				calibWindow.y = calib.y - TRACK_ACQUISITION_WINDOW/2;
				calibWindow.w = TRACK_ACQUISITION_WINDOW;
				calibWindow.h = TRACK_ACQUISITION_WINDOW;
				camera.setWindow(calibWindow);
				camera.requestFrame(); //queue calib frame, pg-comment
				log(pat_health_port, textFileOut,  "In main.cpp phase CL_INIT - Double window tracking set up! Queued", camera.queuedCount, "requests");
				*/
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
				laserOff(fpga_map_request_port, 0); //TBR request number argument, turn calibration laser off for beacon
				camera.config->expose_us.write(beaconExposure); //set cam to beacon exposure, beaconExposure is beacon exposure, pg
				camera.setWindow(beaconWindow);
				camera.requestFrame(); //queue beacon frame, pg-comment
				if(camera.waitForFrame())
				{
					Image frame(camera, textFileOut, pat_health_port, track.beaconSmoothing);

					//save image debug telemetry
					std::string nameTag = std::string("CL_CALIB_DEBUG");
					std::string imageFileName = timeStamp() + std::string("_") + nameTag + std::string("_exp_") + std::to_string(camera.config->expose_us.read()) + std::string(".png");
					log(pat_health_port, textFileOut, "In main.cpp phase CL_CALIB - Saving image telemetry as: ", imageFileName);
					frame.savePNG(imageFileName);

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
										// If running open-loop]
										/*
										if(openLoop)
										{
											double setPointX = 2*((CAMERA_WIDTH/2) + calibration.centerOffsetX) - beacon.x;
											double setPointY = 2*((CAMERA_HEIGHT/2) + calibration.centerOffsetY) - beacon.y;
											track.controlOpenLoop(fsm, setPointX, setPointY);
										}
										*/
										// If sending beacon angle errors to the bus adcs
										//standard sampling frequency is about 1/(40ms) = 25Hz, reduced 25x to 1Hz (TBR - Sychronization)
										if(sendBusFeedback && (i % 25 == 0))
										{
											error_angles bus_feedback = centroid2angles(beacon.x, beacon.y);
											log(pat_health_port, textFileOut, "In main.cpp phase CL_BEACON - Sending Bus Feedback Error Angles: ",
											"(X,Y) = (",bus_feedback.angle_x_radians,", ",bus_feedback.angle_y_radians,"), ",
											"from beacon centroid: (cx,cy) = (",beacon.x,", ",beacon.y,")");
											send_packet_tx_adcs(tx_packets_port, bus_feedback.angle_x_radians, bus_feedback.angle_y_radians);
										}
									}
									else
									{
										log(pat_health_port, textFileOut,  "In main.cpp phase CL_BEACON - Rapid beacon spot property change: ",
										"(propertyDifference = ", propertyDifference, ") >= (TRACK_MAX_SPOT_DIFFERENCE = ",  TRACK_MAX_SPOT_DIFFERENCE, "). ",
										"Prev: [ x = ", beacon.x, ", y = ", beacon.y, ", pixelCount = ", beacon.pixelCount, ", valueMax = ", beacon.valueMax,
										"New: [ x = ", frame.area.x + spot.x, ", y = ", frame.area.y + spot.y, ", pixelCount = ", spot.pixelCount, ", valueMax = ", spot.valueMax);
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

					/*
					// Request new frame
					camera.setWindow(beaconWindow);
					//camera.config->gain_dB.write(beaconGain);
					beaconExposure = track.controlExposure(frame, beaconExposure); //control beacon exposure, pg
					camera.config->expose_us.write(beaconExposure); //set frame exposure, pg
					camera.requestFrame(); //queue beacon frame, pg-comment
					// Next up is calibration laser frame
					*/
					phase = CL_CALIB;
				}
				else
				{
					log(pat_health_port, textFileOut,  "In main.cpp phase CL_BEACON - camera.waitForFrame() Failed! Transitioning to CL_INIT. camera.error: ", camera.error);
					phase = CL_INIT;
				}
				break;

			// Process new frame of calib laser spot
			case CL_CALIB:
				laserOn(fpga_map_request_port, 0); //TBR request number argument, turn calibration laser on for calib
				camera.config->expose_us.write(calibExposure); //set cam to calib exposure, pg
				camera.setWindow(calibWindow);
				camera.requestFrame(); //queue calib frame, pg-comment
				if(camera.waitForFrame())
				{
					Image frame(camera, textFileOut, pat_health_port, calibration.smoothing);
					
					//save image debug telemetry
					std::string nameTag = std::string("CL_CALIB_DEBUG");
					std::string imageFileName = timeStamp() + std::string("_") + nameTag + std::string("_exp_") + std::to_string(camera.config->expose_us.read()) + std::string(".png");
					log(pat_health_port, textFileOut, "In main.cpp phase CL_CALIB - Saving image telemetry as: ", imageFileName);
					frame.savePNG(imageFileName);

					if(frame.histBrightest > CALIB_MIN_BRIGHTNESS/4)
					{
						cl_calib_num_groups = frame.performPixelGrouping();
						if(cl_calib_num_groups > 0)
						{
							spotIndex = track.findSpotCandidate(frame, calib, &propertyDifference);
							if(spotIndex >= 0)
							{
								if(frame.groups[spotIndex].valueMax > CALIB_MIN_BRIGHTNESS/4)
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
										double setPointX = 2*((CAMERA_WIDTH/2) + calibration.centerOffsetX) - beacon.x;
										double setPointY = 2*((CAMERA_HEIGHT/2) + calibration.centerOffsetY) - beacon.y;
										track.control(fsm, calib.x, calib.y, setPointX, setPointY);
										if(i % 100 == 0){ //standard sampling frequency is about 1/(40ms) = 25Hz, reduced 100x to ~1/(400ms) = 2.5Hz
											// Save for CSV
											time_point<steady_clock> now = steady_clock::now();
											duration<double> diff = now - beginTime;
											csvData.insert(make_pair(diff.count(), CSVdata(beacon.x, beacon.y, beaconExposure,
												calib.x, calib.y, setPointX, setPointY, calibExposure)));
											//CSVdata members: double bcnX, bcnY, bcnExp, calX, calY, calSetX, calSetY, calExp;
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
										// Try forced FSM SPI transfer? Maybe data corruption
										// fsm.forceTransfer();
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
									// Try forced FSM SPI transfer? Maybe data corruption
									// fsm.forceTransfer();
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
								// Try forced FSM SPI transfer? Maybe data corruption
								// fsm.forceTransfer();
							}
						}
											
					}
					else
					{
						log(pat_health_port, textFileOut, "In main.cpp phase CL_CALIB - Switching Failure: ",
						"(frame.histBrightest = ", frame.histBrightest, ") <= (CALIB_MIN_BRIGHTNESS/4 = ", CALIB_MIN_BRIGHTNESS/4,")");						

						exit(-1);
						
						if(haveCalibKnowledge)
						{						
							log(pat_health_port, textFileOut,  "In main.cpp phase CL_CALIB - Calib spot vanished!");
							haveCalibKnowledge = false;
							// Try forced FSM SPI transfer? Maybe data corruption
							// fsm.forceTransfer();
						}
					}
					/*
					// Request new frame
					camera.setWindow(calibWindow);
					//camera.config->gain_dB.write(calibGain);
					camera.config->expose_us.write(calibExposure); //set frame exposure, pg
					camera.requestFrame(); //queue calib frame, pg-comment
					// Next up is beacon laser frame
					*/
					phase = CL_BEACON;
				}
				else
				{
					log(pat_health_port, textFileOut,  "In main.cpp phase CL_CALIB - camera.waitForFrame() Failed! Transitioning to CL_INIT. camera.error: ", camera.error);
					phase = CL_INIT;
				}
				break;

			// Graceful failure mode for when beacon is not detected: command the FSM to point the laser straight & rely on bus pointing.
			case STATIC_POINT:
				laserOn(fpga_map_request_port, 0); //TBR request number argument, turn calibration laser off for acquistion
				if(camera.waitForFrame())
				{
					Image frame(camera, textFileOut, pat_health_port, calibration.smoothing);
					if(frame.histBrightest > CALIB_MIN_BRIGHTNESS/4 &&
					   frame.performPixelGrouping() > 0 && (
					   spotIndex = track.findSpotCandidate(frame, calib, &propertyDifference)) >= 0 &&
					   frame.groups[spotIndex].valueMax > CALIB_MIN_BRIGHTNESS/4)
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
							// Set Point is defined as the center with any measured biases
							double setPointX = 2*((CAMERA_WIDTH/2) + calibration.centerOffsetX);
							double setPointY = 2*((CAMERA_HEIGHT/2) + calibration.centerOffsetY);
							track.control(fsm, calib.x, calib.y, setPointX, setPointY);
							if(i % 100 == 0){ //standard sampling frequency is about 1/(40ms) = 25Hz, reduced 10x to ~1/(400ms) = 2.5Hz
								// Save for CSV
								time_point<steady_clock> now = steady_clock::now();
								duration<double> diff = now - beginTime;
								csvData.insert(make_pair(diff.count(), CSVdata(0, 0, 0,
									calib.x, calib.y, setPointX, setPointY, calibExposure)));
								//CSVdata members: double bcnX, bcnY, bcnExp, calX, calY, calSetX, calSetY, calExp;
							}
						}
						else
						{
							if(haveCalibKnowledge) log(pat_health_port, textFileOut,  "In main.cpp phase STATIC_POINT - Rapid calib spot property change: ",
									"(propertyDifference = ", propertyDifference, ") >= (TRACK_MAX_SPOT_DIFFERENCE = ",  TRACK_MAX_SPOT_DIFFERENCE, "). ",
									"Prev: [ x = ", calib.x, ", y = ", calib.y, ", pixelCount = ", calib.pixelCount, ", valueMax = ", calib.valueMax,
									"New: [ x = ", frame.area.x + spot.x, ", y = ", frame.area.y + spot.y, ", pixelCount = ", spot.pixelCount, ", valueMax = ", spot.valueMax);
							haveCalibKnowledge = false;
						}
					}
					else
					{
						if(haveCalibKnowledge)
						{
							log(pat_health_port, textFileOut,  "In main.cpp phase STATIC_POINT - Calib spot vanished! Try FSM SPI forceTransfer().");
							// Try forced FSM SPI transfer? Maybe data corruption
							fsm.forceTransfer();
						}
						haveCalibKnowledge = false;
					}

					// Request new frame
					camera.setWindow(calibWindow);
					//camera.config->gain_dB.write(calibGain);
					camera.config->expose_us.write(calibExposure); //set frame exposure, pg
					camera.requestFrame(); //queue calib frame, pg-comment
				}
				else
				{
					log(pat_health_port, textFileOut, "In main.cpp phase STATIC_POINT - camera.waitForFrame() Failed! Trying again. camera.error: ", camera.error);
				}
				break;

			// Control in open-loop, sampling only beacon spot!
			case OPEN_LOOP:
				laserOff(fpga_map_request_port, 0); //TBR request number argument, turn calibration laser off for open-loop
				if(camera.waitForFrame())
				{
					Image frame(camera, textFileOut, pat_health_port, track.beaconSmoothing);
					if(frame.histBrightest > TRACK_ACQUISITION_BRIGHTNESS &&
					   frame.performPixelGrouping() > 0 && (
					   spotIndex = track.findSpotCandidate(frame, beacon, &propertyDifference)) >= 0 &&
					   frame.groups[spotIndex].valueMax > TRACK_ACQUISITION_BRIGHTNESS)
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
							// Control pointing in open-loop
							calib.x = 2*((CAMERA_WIDTH/2) + calibration.centerOffsetX) - beacon.x;
							calib.y = 2*((CAMERA_HEIGHT/2) + calibration.centerOffsetY) - beacon.y;
							track.controlOpenLoop(fsm, calib.x, calib.y);
							if(i % 100 == 0){ //standard sampling frequency is about 1/(40ms) = 25Hz, reduced 10x to ~1/(400ms) = 2.5Hz
								// Save for CSV
								time_point<steady_clock> now = steady_clock::now();
								duration<double> diff = now - beginTime;
								csvData.insert(make_pair(diff.count(), CSVdata(beacon.x, beacon.y, beaconExposure,
									calib.x, calib.y, beacon.x, beacon.y, calibExposure)));
								//CSVdata members: double bcnX, bcnY, bcnExp, calX, calY, calSetX, calSetY, calExp;
							}
						}
						else
						{
							log(pat_health_port, textFileOut,  "In main.cpp phase CL_BEACON - Rapid beacon spot property change: ",
								"(propertyDifference = ", propertyDifference, ") >= (TRACK_MAX_SPOT_DIFFERENCE = ",  TRACK_MAX_SPOT_DIFFERENCE, "). ",
								"Prev: [ x = ", beacon.x, ", y = ", beacon.y, ", pixelCount = ", beacon.pixelCount, ", valueMax = ", beacon.valueMax,
								"New: [ x = ", frame.area.x + spot.x, ", y = ", frame.area.y + spot.y, ", pixelCount = ", spot.pixelCount, ", valueMax = ", spot.valueMax);
							haveBeaconKnowledge = false;
						}
					}
					else
					{
						if(haveBeaconKnowledge) log(pat_health_port, textFileOut,  "In main.cpp phase OPEN_LOOP - Beacon spot vanished!");
						haveBeaconKnowledge = false;
					}

					// Request new frame
					camera.setWindow(beaconWindow);
					camera.requestFrame(); //queue beacon frame, pg-comment
				}
				else
				{
					log(pat_health_port, textFileOut, "In main.cpp phase OPEN_LOOP - camera.waitForFrame() Failed! Trying again. camera.error: ", camera.error);
				}				
				break;

			// Fail-safe
			default:
				log(pat_health_port, textFileOut,  "In main.cpp phase CL_CALIB - Unknown phase encountered in switch structure. Resetting to START...");
				phase = START;
				break;
		}
	}
	// END of main PAT loop

	log(pat_health_port, textFileOut,  "In main.cpp - Saving telemetry files...");

	ofstream out(dataFileName);
	for(const auto& x : csvData)
	{
		out << x.first << "," << x.second.bcnX << "," << x.second.bcnY << "," << x.second.bcnExp << ","
		<< x.second.calX << "," << x.second.calY << "," << x.second.calSetX << "," << x.second.calSetY << "," << x.second.calExp << endl;
		//x.first = time since beginTime
		//	CSVdata members: double bcnX, bcnY, bcnExp, calX, calY, calSetX, calSetY, calExp;
	}
	out.close(); //close data file

	textFileOut.close(); //close text file

	log(pat_health_port, textFileOut,  "In main.cpp - Exiting Process.");

	return 0;
}
