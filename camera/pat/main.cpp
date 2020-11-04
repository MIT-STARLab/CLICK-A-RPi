#include <unistd.h>
#include <atomic>
#include <csignal>
#include <chrono>
#include <thread>
//#include <pigpiod_if2.h> //nonflight
#include <zmq.hpp>

#include "log.h"
#include "fsm.h"
#include "camera.h"
#include "processing.h"
#include "tracking.h"
#include "calibration.h"
//#include "link.h" //nonflight
#include "fpga.h"
#include "zhelpers.hpp"

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

string timeStamp() 	//Get date and time for telemetry file names
{
	time_t     now = time(0);
	struct tm  tstruct;
	char       dateTime[80];
	tstruct = *localtime(&now);
	strftime(dateTime, sizeof(dateTime), "%Y-%m-%d-%H-%M-%S", &tstruct);
	return string(dateTime);
}

void logImage(string nameTag, Camera& cameraObj, ofstream& textFileIn)
{
	cameraObj.requestFrame(); //queue frame
	if(cameraObj.waitForFrame())
	{
		const string imageFileName = timeStamp() + string("_") + nameTag + string(".bmp");
		Image frame(cameraObj, textFileIn);
		log(std::cout, textFileIn, "Saving image telemetry as: ", imageFileName);
		frame.saveBMP(imageFileName);
	}
	else
	{
		log(std::cerr, textFileIn, "Error: waitForFrame");
	}
}

atomic<bool> stop(false);

//-----------------------------------------------------------------------------
int main(int argc, char** argv)
//-----------------------------------------------------------------------------
{
	//zeromq init socket
	zmq::context_t context(1); //zmq context
	zmq::socket_t sender(context, ZMQ_PUSH);
	sender.bind("tcp://*5555");

	std::string textFileName = timeStamp() + string("_logs.txt"); //used for text telemetry
	std::string dataFileName = timeStamp() + string("_data.csv"); //used by csv data file generation

	zmq::message_t textFileMsg(sizeof(textFileName));
	zmq::message_t dataFileMsg(sizeof(dataFileName));
	memcpy (textFileMsg.data(), &textFileName, sizeof(textFileName));
	memcpy (dataFileMsg.data(), &dataFileName, sizeof(dataFileName));
	sender.send(textFileMsg);
	sender.send(dataFileMsg);

  //Generate text telemetry file, pg
	ofstream textFileOut; //stream for text telemetry
	textFileOut.open(textFileName, ios::app); //create text file and open for writing

	// Synchronization
	enum Phase { START, CALIBRATION, ACQUISITION, STATIC_POINT, OPEN_LOOP, CL_INIT, CL_BEACON, CL_CALIB };

	// Connection to GUI, testing only
	/*
	log(std::cout, textFileOut, "Connecting to GUI"); //testing only
	Link link("http://10.0.5.1:3000"); //testing only
	*/
	/*
	// not for flight, GPIO init
	int gpioHandle = pigpio_start(0, 0);
	if(gpioHandle < 0)
	{
		log(std::cerr, textFileOut,  "Failed to initialize GPIO!");
		exit(1);
	}
	*/

	// Hardware init
	Camera camera(textFileOut);
	FPGA fpga(textFileOut);
	FSM fsm(80, 129, 200, textFileOut, fgpa);
	//nonflight args: (gpioHandle, SPI0_CE0, PWM0, GPIO6, 80, 129, 200, textFileOut, fgpa)
	Calibration calibration(camera, fsm, textFileOut);
	Tracking track(camera, calibration, textFileOut);

	// Killing app handler
	signal(SIGINT, [](int signum) { stop = true; });

	// Tracking variables
	Phase phase = START;
	Group beacon, calib;
	AOI beaconWindow, calibWindow;
	int beaconExposure = 0, spotIndex = 0, calibExposure = 0;
	int beaconGain = 0, calibGain = 0;
	bool haveBeaconKnowledge = false, haveCalibKnowledge = false;
	double propertyDifference = 0;

	// not for flight, Offset changes from GUI (user-input)
	/*
	atomic<int> centerOffsetX(5), centerOffsetY(32);
	link.on("offsets", [&](sio::event& ev)
	{
		centerOffsetX = ev.get_message()->get_map()["x"]->get_int();
		centerOffsetY = ev.get_message()->get_map()["y"]->get_int();
		log(std::cout, textFileOut,  "Center offset updated to [", centerOffsetX, ",", centerOffsetY, "]");
	});
	*/
	int centerOffsetX = 0; //will need an automatic function for generating these: they depend on mechanical alignment
	int centerOffsetY = 0; //findOffset()

	// Open-loop vs. closed-loop run mode; currently controlled via giving any argument to run in open-loop
	bool openLoop = false;
	if(argc != 1)
	{
		openLoop = true;
		log(std::cout, textFileOut,  "Running in OPEN-LOOP mode");
	}

	// CSV data saving, TBD
	time_point<steady_clock> beginTime;
	map<double, CSVdata> csvData;

	//Beacon Loss Timing, pg
	time_point<steady_clock> startBeaconLoss; //pg
	duration<double> waitBeaconLoss(5.0); //wait time before switching back to ACQUISITION, pg

	// Main loop
	for(int i = 1; ; i++)
	{
		switch(phase)
		{
			case START:
				fpga.laserOff(); //ensure calibration laser off
				//logAndConfirm("Start calibration with Enter..."); //testing only
				log(std::cout, textFileOut, "Calibration Starting..."); //testing only
				fpga.laserOn(); //turn calibration laser on
				phase = CALIBRATION;
				break;

			// Calibration phase, internal laser has to be turned on, ran just once
			case CALIBRATION:
				//fsm.enableAmp(); nonflight
				if(calibration.run(calib)) //sets calib exposure, pg-comment
				{
					calibExposure = camera.config->expose_us.read(); //save calib exposure, pg
					log(std::cout, textFileOut, "Calibration complete: ", calibExposure, " us");
					logImage(string("CALIBRATION"), camera, textFileOut); //save image
					fpga.laserOff(); //turn calibration laser off for acquistion
					phase = ACQUISITION;
				}
				else
				{
					log(std::cerr, textFileOut,  "Calibration failed!");
					phase = START;
				}
				break;

			// Beacon acquisition phase, internal laser has to be off!
			case ACQUISITION:
				//logAndConfirm("Start beacon acquisition with Enter..."); //test only
				log(std::cout, textFileOut, "Beacon Acquisition Beginning");
				if(track.runAcquisition(beacon) && beacon.pixelCount > MIN_PIXELS_PER_GROUP) //sets beacon exposure, pg-comment
				{
					// Acquisition passed!
					haveBeaconKnowledge = true;
					haveCalibKnowledge = false;
					// Save all important parameters
					beaconExposure = camera.config->expose_us.read(); //save beacon exposure, pg-comment
					beaconGain = camera.config->gain_dB.read();
					calibGain = calibration.gainForExposure(beaconExposure);
					log(std::cout, textFileOut,  "Acquisition complete:", beaconExposure, "us ; beacon:",
						beaconGain, "dB smoothing", track.beaconSmoothing, "; calib:",
						calibGain, "dB smoothing", calibration.smoothing);
					logImage(string("ACQUISITION"), camera, textFileOut); //save image
					// Set initial pointing in open-loop
					calib.x = 2*((CAMERA_WIDTH/2) + centerOffsetX) - beacon.x;
					calib.y = 2*((CAMERA_HEIGHT/2) + centerOffsetY) - beacon.y;
					track.controlOpenLoop(fsm, calib.x, calib.y);
					// logAndConfirm("Start open-loop tracking with Enter...");
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
					log(std::cout, textFileOut, "Beacon Acquisition Failed! Transitioning to Static Pointing Mode...");
					phase = STATIC_POINT; 
				}
				break;

			// Graceful failure mode for when beacon is not detected: command the FSM to point the laser straight & rely on bus pointing.
			case STATIC_POINT:
				if(camera.waitForFrame())
				{
					Image frame(camera, textFileOut, calibration.smoothing);
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
							// calib.valueMax = spot.valueMax;
							// calib.valueSum = spot.valueSum;
							// calib.pixelCount = spot.pixelCount;
							track.updateTrackingWindow(frame, spot, calibWindow);
							// Set Point is defined as the center with any measured biases
							double setPointX = 2*((CAMERA_WIDTH/2) + centerOffsetX);
							double setPointY = 2*((CAMERA_HEIGHT/2) + centerOffsetY);
							track.control(fsm, calib.x, calib.y, setPointX, setPointY);
							if(i % 10 == 0){ //standard sampling frequency is about 1/(40ms) = 25Hz, reduced 10x to ~1/(400ms) = 2.5Hz
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
							if(haveCalibKnowledge) log(std::cout, textFileOut,  "Panic, rapid calib spot property change, old", calib.x, calib.y, calib.pixelCount,
								calib.valueMax, "new", frame.area.x + spot.x, frame.area.y + spot.y, spot.pixelCount, spot.valueMax);
							haveCalibKnowledge = false;
						}
					}
					else
					{
						if(haveCalibKnowledge)
						{
							log(std::cout, textFileOut,  "Panic, calib spot vanished!");
							// Try forced FSM SPI transfer? Maybe data corruption
							fsm.forceTransfer();
						}
						haveCalibKnowledge = false;
					}
					// nonflight, Send image to GUI
					//link.setCalib(frame);

					// Request new frame
					camera.setWindow(calibWindow);
					//camera.config->gain_dB.write(calibGain);
					camera.config->expose_us.write(calibExposure); //set frame exposure, pg
					camera.requestFrame(); //queue calib frame, pg-comment
				}
				else
				{
					log(std::cout, textFileOut, "camera.waitForFrame() Failed! Trying again. camera.error: ", camera.error);
				}
				break;

			// Control in open-loop, sampling only beacon spot!
			case OPEN_LOOP:
				if(camera.waitForFrame())
				{
					Image frame(camera, textFileOut, track.beaconSmoothing);
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
							calib.x = 2*((CAMERA_WIDTH/2) + centerOffsetX) - beacon.x;
							calib.y = 2*((CAMERA_HEIGHT/2) + centerOffsetY) - beacon.y;
							track.controlOpenLoop(fsm, calib.x, calib.y);
						}
						else
						{
							if(haveBeaconKnowledge) log(std::cout, textFileOut,  "Panic, rapid beacon spot property change, old", beacon.x, beacon.y, beacon.pixelCount,
								beacon.valueMax, "new", frame.area.x + spot.x, frame.area.y + spot.y, spot.pixelCount, spot.valueMax);
							haveBeaconKnowledge = false;
						}
					}
					else
					{
						if(haveBeaconKnowledge) log(std::cout, textFileOut,  "Panic, beacon spot vanished!");
						haveBeaconKnowledge = false;
					}
					//nonflight: Send image to GUI
					//link.setBeacon(frame);

					// Request new frame
					camera.setWindow(beaconWindow);
					camera.requestFrame(); //queue beacon frame, pg-comment
				}
				else
				{
					log(std::cout, textFileOut, "camera.waitForFrame() Failed! Trying again. camera.error: ", camera.error);
				}				
				break;

			// Initialize closed-loop double window tracking
			case CL_INIT:
				camera.ignoreNextFrames(camera.queuedCount);
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
				// Initial values will be uncertain
				calib.pixelCount = 0;
				camera.setWindow(calibWindow);
				camera.requestFrame(); //queue calib frame, pg-comment
				log(std::cout, textFileOut,  "Double window tracking set up! Queued", camera.queuedCount, "requests");
				//logAndConfirm("Turn on calibration laser and begin tracking with Enter..."); //test only
				fpga.laserOn(); //turn on calibration laser
				log(std::cout, textFileOut, "Calibration Laser On and Tracking Beginning");
				// Next up is beacon spot frame
				phase = CL_BEACON;
				// Save time
				beginTime = steady_clock::now();
				break;

			// Process new frame of beacon spot
			case CL_BEACON:
				if(camera.waitForFrame())
				{
					Image frame(camera, textFileOut, track.beaconSmoothing);
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
							// If running open-loop
							if(openLoop)
							{
								double setPointX = 2*((CAMERA_WIDTH/2) + centerOffsetX) - beacon.x;
								double setPointY = 2*((CAMERA_HEIGHT/2) + centerOffsetY) - beacon.y;
								track.controlOpenLoop(fsm, setPointX, setPointY);
							}
						}
						else
						{
							if(haveBeaconKnowledge) //Rapid spot change -> loss scenario, pg
							{
								log(std::cout, textFileOut,  "Panic, rapid beacon spot property change, old", beacon.x, beacon.y, beacon.pixelCount,
								beacon.valueMax, "new", frame.area.x + spot.x, frame.area.y + spot.y, spot.pixelCount, spot.valueMax);
								haveBeaconKnowledge = false; // This variable is false if timeout has started, pg
								startBeaconLoss = steady_clock::now(); // Record time of Loss, pg
						  }
						}
					}
					else
					{
						if(haveBeaconKnowledge) // Beacon Loss Scenario, pg
						{
							log(std::cout, textFileOut,  "Beacon spot vanished! Timeout started..."); //pg
							haveBeaconKnowledge = false;
							startBeaconLoss = steady_clock::now(); // Record time of Loss, pg
						}
					}

					if(!haveBeaconKnowledge) //Check timeout and return to acquisition if loss criterion met, pg
					{
						time_point<steady_clock> checkBeaconLoss = steady_clock::now(); // Record current time, pg
						duration<double> elapsedBeaconLoss = checkBeaconLoss - startBeaconLoss; // Calculate time since beacon loss, pg
						if(elapsedBeaconLoss > waitBeaconLoss) //pg
						{
							phase = ACQUISITION; // Beacon completely lost, return to ACQUISITION, pg
							break; //pg
						}
					}

					// nonflight, Send image to GUI
					//link.setBeacon(frame);

					// Request new frame
					camera.setWindow(beaconWindow);
					//camera.config->gain_dB.write(beaconGain);
					beaconExposure = track.controlExposure(frame, beaconExposure); //control beacon exposure, pg
					camera.config->expose_us.write(beaconExposure); //set frame exposure, pg
					camera.requestFrame(); //queue beacon frame, pg-comment
					// Next up is calibration laser frame
					phase = CL_CALIB;
				}
				else
				{
					log(std::cerr, textFileOut,  "camera.waitForFrame() Failed! Transitioning to CL_INIT. camera.error: ", camera.error);
					phase = CL_INIT;
				}
				break;

			// Process new frame of calib laser spot
			case CL_CALIB:
				if(camera.waitForFrame())
				{
					Image frame(camera, textFileOut, calibration.smoothing);
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
							// calib.valueMax = spot.valueMax;
							// calib.valueSum = spot.valueSum;
							// calib.pixelCount = spot.pixelCount;
							track.updateTrackingWindow(frame, spot, calibWindow);
							// Control in closed loop!
							double setPointX = 2*((CAMERA_WIDTH/2) + centerOffsetX) - beacon.x;
							double setPointY = 2*((CAMERA_HEIGHT/2) + centerOffsetY) - beacon.y;
							if(!openLoop)
							{
								track.control(fsm, calib.x, calib.y, setPointX, setPointY);
							}
							if(i % 10 == 0){ //standard sampling frequency is about 1/(40ms) = 25Hz, reduced 10x to ~1/(400ms) = 2.5Hz
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
							if(haveCalibKnowledge) log(std::cout, textFileOut,  "Panic, rapid calib spot property change, old", calib.x, calib.y, calib.pixelCount,
								calib.valueMax, "new", frame.area.x + spot.x, frame.area.y + spot.y, spot.pixelCount, spot.valueMax);
							haveCalibKnowledge = false;
						}
					}
					else
					{
						if(haveCalibKnowledge)
						{
							log(std::cout, textFileOut,  "Panic, calib spot vanished!");
							// Try forced FSM SPI transfer? Maybe data corruption
							fsm.forceTransfer();
						}
						haveCalibKnowledge = false;
					}
					// nonflight, Send image to GUI
					//link.setCalib(frame);

					// Request new frame
					camera.setWindow(calibWindow);
					//camera.config->gain_dB.write(calibGain);
					camera.config->expose_us.write(calibExposure); //set frame exposure, pg
					camera.requestFrame(); //queue calib frame, pg-comment
					// Next up is beacon laser frame
					phase = CL_BEACON;
				}
				else
				{
					log(std::cerr, textFileOut,  "camera.waitForFrame() Failed! Transitioning to CL_INIT. camera.error: ", camera.error);
					phase = CL_INIT;
				}
				break;

			// Fail-safe
			default:
				log(std::cerr, textFileOut,  "Unknown phase encountered in switch structure. Resetting to START...");
				phase = START;
				break;
		}

		// Debug periodic console update
		if(i % 200 == 0)
		{
			if(phase == OPEN_LOOP)
			{
				log(std::cout, textFileOut,  haveBeaconKnowledge ? "Beacon is at" : "No idea where beacon is",
					"[", beacon.x, ",", beacon.y, "]");
			}
			else
			{
				if(beaconExposure == TRACK_MIN_EXPOSURE) log(std::cout, textFileOut,  "Minimum beacon exposure reached!"); //notification when exposure limits reached, pg
				if(beaconExposure == TRACK_MAX_EXPOSURE) log(std::cout, textFileOut,  "Maximum beacon exposure reached!");

				log(std::cout, textFileOut,  haveBeaconKnowledge ? "Beacon is at" : "No idea where beacon is",
					"[", beacon.x, ",", beacon.y, ", exp = ", beaconExposure, "]",
					haveCalibKnowledge ? "Calib is at" : "No idea where calib is",
					"[", calib.x, ",", calib.y, ", exp = ", calibExposure, "]"); //included exposure updates, pg
			}
			i = 0;
		}

		//not for flight, GUI update (manages FPS itself)
		//link.sendUpdate(haveBeaconKnowledge, haveCalibKnowledge, beacon.x, beacon.y, calib.x, calib.y, track.actionX, track.actionY);

		// Allow exit with Ctrl-C
		if(stop) break;
	}

	log(std::cout, textFileOut,  "\nFinishing...");
	//fsm.disableAmp(); not for flight, zero reset in destructor

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

	return 0;
}
