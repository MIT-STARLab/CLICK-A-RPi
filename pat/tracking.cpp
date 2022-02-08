// CLICK A Tracking algorithms
// Authors: Ondrej Cierny, Peter Grenfell
#include "tracking.h"
#include <cmath>

//Constructor
//-----------------------------------------------------------------------------
Tracking::Tracking(Camera& c, Calibration& calib, std::ofstream &fileStreamIn, zmq::socket_t &pat_status_port_in, zmq::socket_t &pat_health_port_in, zmq::socket_t& pat_control_port_in, std::vector<zmq::pollitem_t>& poll_pat_control_in): 
camera(c), calibration(calib), fileStream(fileStreamIn), pat_status_port(pat_status_port_in), pat_health_port(pat_health_port_in), pat_control_port(pat_control_port_in), poll_pat_control(poll_pat_control_in), actionX(0), actionY(0)
//-----------------------------------------------------------------------------
{
	if(!getTrackParams()){
		//assign default parameters if CSV parameter retrieval fails
		log(pat_health_port, fileStream, "In tracking.cpp - Tracking: using default parameters:");
		
		//Set names
		trackParams[IDX_TRACK_GUESS_EXPOSURE].name = "TRACK_GUESS_EXPOSURE";
		trackParams[IDX_TRACK_MIN_EXPOSURE].name = "TRACK_MIN_EXPOSURE";
		trackParams[IDX_TRACK_MAX_EXPOSURE].name = "TRACK_MAX_EXPOSURE";
		trackParams[IDX_TRACK_ACQUISITION_EXP_INCREMENT].name = "TRACK_ACQUISITION_EXP_INCREMENT";
		trackParams[IDX_TRACK_ACQUISITION_BRIGHTNESS].name = "TRACK_ACQUISITION_BRIGHTNESS";
		trackParams[IDX_TRACK_ACQUISITION_WINDOW].name = "TRACK_ACQUISITION_WINDOW";
		trackParams[IDX_TRACK_GOOD_PEAKTOMAX_DISTANCE].name = "TRACK_GOOD_PEAKTOMAX_DISTANCE";
		trackParams[IDX_TRACK_HAPPY_BRIGHTNESS].name = "TRACK_HAPPY_BRIGHTNESS";
		trackParams[IDX_TRACK_TUNING_TOLERANCE].name = "TRACK_TUNING_TOLERANCE";
		trackParams[IDX_TRACK_TUNING_EXP_DIVIDER].name = "TRACK_TUNING_EXP_DIVIDER";
		trackParams[IDX_TRACK_EXP_CONTROL_TOLERANCE].name = "TRACK_EXP_CONTROL_TOLERANCE";
		trackParams[IDX_TRACK_EXP_CONTROL_DIVIDER].name = "TRACK_EXP_CONTROL_DIVIDER";
		trackParams[IDX_TRACK_WINDOW_SIZE_TOLERANCE].name = "TRACK_WINDOW_SIZE_TOLERANCE";
		trackParams[IDX_TRACK_MAX_SPOT_DIFFERENCE].name = "TRACK_MAX_SPOT_DIFFERENCE";
		trackParams[IDX_TRACK_MIN_SPOT_LIMIT].name = "TRACK_MIN_SPOT_LIMIT";
		trackParams[IDX_TRACK_CONTROL_I].name = "TRACK_CONTROL_I";
		trackParams[IDX_TRACK_CONTROL_MAX_TS_MS].name = "TRACK_CONTROL_MAX_TS_MS";
		trackParams[IDX_TRACK_SAFE_DISTANCE_ALLOW].name = "TRACK_SAFE_DISTANCE_ALLOW";
		trackParams[IDX_TRACK_SAFE_DISTANCE_PANIC].name = "TRACK_SAFE_DISTANCE_PANIC";

		//Set default parameters
		trackParams[IDX_TRACK_GUESS_EXPOSURE].parameter = TRACK_GUESS_EXPOSURE; //implemented
		trackParams[IDX_TRACK_MIN_EXPOSURE].parameter = TRACK_MIN_EXPOSURE; //implemented
		trackParams[IDX_TRACK_MAX_EXPOSURE].parameter = TRACK_MAX_EXPOSURE; //implemented
		trackParams[IDX_TRACK_ACQUISITION_EXP_INCREMENT].parameter = TRACK_ACQUISITION_EXP_INCREMENT; //implemented
		trackParams[IDX_TRACK_ACQUISITION_BRIGHTNESS].parameter = TRACK_ACQUISITION_BRIGHTNESS; //implemented
		trackParams[IDX_TRACK_ACQUISITION_WINDOW].parameter = TRACK_ACQUISITION_WINDOW; //implemented
		trackParams[IDX_TRACK_GOOD_PEAKTOMAX_DISTANCE].parameter = TRACK_GOOD_PEAKTOMAX_DISTANCE; //implemented
		trackParams[IDX_TRACK_HAPPY_BRIGHTNESS].parameter = TRACK_HAPPY_BRIGHTNESS; //implemented
		trackParams[IDX_TRACK_TUNING_TOLERANCE].parameter = TRACK_TUNING_TOLERANCE; //implemented
		trackParams[IDX_TRACK_TUNING_EXP_DIVIDER].parameter = TRACK_TUNING_EXP_DIVIDER; //implemented
		trackParams[IDX_TRACK_EXP_CONTROL_TOLERANCE].parameter = TRACK_EXP_CONTROL_TOLERANCE; //implemented
		trackParams[IDX_TRACK_EXP_CONTROL_DIVIDER].parameter = TRACK_EXP_CONTROL_DIVIDER; //implemented
		trackParams[IDX_TRACK_WINDOW_SIZE_TOLERANCE].parameter = TRACK_WINDOW_SIZE_TOLERANCE; //implemented
		trackParams[IDX_TRACK_MAX_SPOT_DIFFERENCE].parameter = TRACK_MAX_SPOT_DIFFERENCE; //implemented
		trackParams[IDX_TRACK_MIN_SPOT_LIMIT].parameter = TRACK_MIN_SPOT_LIMIT; //implemented
		trackParams[IDX_TRACK_CONTROL_I].parameter = TRACK_CONTROL_I; //implemented
		trackParams[IDX_TRACK_CONTROL_MAX_TS_MS].parameter = (int) 1000*TRACK_CONTROL_MAX_TS; //implemented
		trackParams[IDX_TRACK_SAFE_DISTANCE_ALLOW].parameter = TRACK_SAFE_DISTANCE_ALLOW; //implemented
		trackParams[IDX_TRACK_SAFE_DISTANCE_PANIC].parameter = TRACK_SAFE_DISTANCE_PANIC; //implemented

		//Display parameters
		for (size_t i = 0; i < NUM_TRACK_PARAMS; i++)
		{
			log(pat_health_port, fileStream, "In tracking.cpp - Tracking: ", trackParams[i].name, ": ", trackParams[i].parameter);
		}			
	}
	track_control_max_ts = (double) trackParams[IDX_TRACK_CONTROL_MAX_TS_MS].parameter/1000;
}

//Load modifiable constant parameters from external CSV file
//-----------------------------------------------------------------------------
bool Tracking::getTrackParams()
//-----------------------------------------------------------------------------
{
	trackParamStruct trackParam; //temp offsetParamStruct for use in the while loop
    ifstream inFile("/root/lib/trackParams.csv"); //our file
    string line;
    int linenum = 0;
	log(pat_health_port, fileStream, "In tracking.cpp - getTrackParams: Retrieving Tracking Parameters from /root/lib/trackParams.csv");
    if(inFile.is_open()){
		while (getline (inFile, line))
		{
			istringstream linestream(line);
			string item;
			//use this to get up to the first comma
			getline(linestream, item, ',');
			trackParam.name = item;
			//convert to a string stream and then put in id.
			getline(linestream, item, ',');
			stringstream ss(item);
			ss >> trackParam.parameter;
			//report read data
			log(pat_health_port, fileStream, "In tracking.cpp - getTrackParams: ", trackParam.name, ": ", trackParam.parameter);
			//add the new data to the list
			trackParams[linenum] = trackParam;
			linenum++;
		}
		if(linenum != NUM_TRACK_PARAMS){
			log(pat_health_port, fileStream, "In tracking.cpp - getTrackParams: /root/lib/trackParams.csv is missing data: ", linenum, " lines found out of ", NUM_TRACK_PARAMS);
            return false;
        } else{
			return true;
		}
	} else{
		log(pat_health_port, fileStream, "In tracking.cpp - getTrackParams: /root/lib/trackParams.csv did not open or doesn't exist.");
		return false;
	}
}

// Sweep through expected power ranges and look for spot
//-----------------------------------------------------------------------------
bool Tracking::runAcquisition(Group& beacon, AOI& beaconWindow, int maxExposure)
//-----------------------------------------------------------------------------
{
	int exposure = trackParams[IDX_TRACK_GUESS_EXPOSURE].parameter, gain = 0, skip = camera.queuedCount, counter = 0, printPeriod = 100;
	uint16_t command;

	//CH Status Msg Telemetry Timing
	time_point<steady_clock> time_prev_status;
	duration<double> period_status(PERIOD_STATUS_MSG_TRACK); //wait time in between status messages
	time_point<steady_clock> check_status; // Record current time
	duration<double> elapsed_time_status; // time since status

	// Skip pre-queued old frames
	camera.ignoreNextFrames(skip);

	camera.setCenteredWindow(beacon.x, beacon.y, beaconWindow.w); //camera.setFullWindow();
	beaconWindow.x = camera.config->aoiStartX.read();
	beaconWindow.y = camera.config->aoiStartY.read();
	beaconWindow.h = beaconWindow.w;
	// camera.config->binningMode.write(cbmBinningHV);
	camera.config->expose_us.write(exposure);
	camera.config->gain_dB.write(gain);
	camera.requestFrame();

	// Try guessed value
	log(pat_health_port, fileStream, "In tracking.cpp Tracking::runAcquisition - Attemping acquisition with exposure = ", exposure, "at ", beacon.x - CAMERA_WIDTH/2, beacon.y - CAMERA_HEIGHT/2, " rel-to-center w/ size ", beaconWindow.w);
	if(camera.waitForFrame())
	{
		Image frame(camera, fileStream, pat_health_port);
		if(verifyFrame(frame, true) && windowAndTune(frame, beacon, beaconWindow, maxExposure)){return true;}
	}

	bool searching_up = true, searching_down = true;
	int exposure_up = exposure, exposure_down = exposure; 
	exposure_up += trackParams[IDX_TRACK_ACQUISITION_EXP_INCREMENT].parameter;
	exposure_down -= trackParams[IDX_TRACK_ACQUISITION_EXP_INCREMENT].parameter; 
	counter++;

	while(searching_up || searching_down){
		// Listen for CMD_END_PAT 		
		zmq::poll(poll_pat_control.data(), 1, 10); // when timeout_ms (the third argument here) is -1, then block until ready to receive (based on: https://ogbe.net/blog/zmq_helloworld.html)
		if(poll_pat_control[0].revents & ZMQ_POLLIN) {
			// received something on the first (only) socket
			command = receive_packet_pat_control(pat_control_port);
			if (command == CMD_END_PAT){
				log(pat_health_port, fileStream, "In tracking.cpp Tracking::runAcquisition - Received CMD_END_PAT");
			  	received_end_pat_cmd = true;
			  	return false; 
			}
			else if (command == CMD_END_PROCESS){
				log(pat_health_port, fileStream, "In tracking.cpp Tracking::runAcquisition - Received CMD_END_PROCESS");
			  	received_end_process_cmd = true;
			  	return false; 
			}
		}

		check_status = steady_clock::now(); // Record current time
		elapsed_time_status = check_status - time_prev_status; // Calculate time since status msg
		if(elapsed_time_status > period_status) //pg
		{
			//log(pat_health_port, fileStream, "In tracking.cpp - Main - Sending STATUS_MAIN"); 
			send_packet_pat_status(pat_status_port, STATUS_MAIN); //send status message
			time_prev_status = steady_clock::now(); // Record time of message						
		}

		//try search up:
		if(exposure_up <= maxExposure){
			if(counter%printPeriod == 0){log(pat_health_port, fileStream, "In tracking.cpp Tracking::runAcquisition - Attemping acquisition with exposure = ", exposure_up);}
			camera.config->expose_us.write(exposure_up);
			camera.requestFrame();
			if(camera.waitForFrame()){
				Image frame(camera, fileStream, pat_health_port);
				if(verifyFrame(frame, (counter%printPeriod == 0)) && windowAndTune(frame, beacon, beaconWindow, maxExposure)){return true;}
			}
			//update exposure:
			exposure_up += trackParams[IDX_TRACK_ACQUISITION_EXP_INCREMENT].parameter;
			counter++;
		} else{
			searching_up = false;
		}		

		// Listen for CMD_END_PAT 		
		zmq::poll(poll_pat_control.data(), 1, 10); // when timeout_ms (the third argument here) is -1, then block until ready to receive (based on: https://ogbe.net/blog/zmq_helloworld.html)
		if(poll_pat_control[0].revents & ZMQ_POLLIN) {
			// received something on the first (only) socket
			command = receive_packet_pat_control(pat_control_port);
			if (command == CMD_END_PAT){
				log(pat_health_port, fileStream, "In tracking.cpp Tracking::runAcquisition - Received CMD_END_PAT");
			  	received_end_pat_cmd = true;
			  	return false; 
			}
			else if (command == CMD_END_PROCESS){
				log(pat_health_port, fileStream, "In tracking.cpp Tracking::runAcquisition - Received CMD_END_PROCESS");
			  	received_end_process_cmd = true;
			  	return false; 
			}
		}
		
		//try search down:
		if(exposure_down >= trackParams[IDX_TRACK_MIN_EXPOSURE].parameter){
			if(counter%printPeriod == 0){log(pat_health_port, fileStream, "In tracking.cpp Tracking::runAcquisition - Attemping acquisition with exposure = ", exposure_down);}
			camera.config->expose_us.write(exposure_down);
			camera.requestFrame();
			if(camera.waitForFrame()){
				Image frame(camera, fileStream, pat_health_port);
				if(verifyFrame(frame, (counter%printPeriod == 0)) && windowAndTune(frame, beacon, beaconWindow, maxExposure)){return true;}
			}
			//update exposure:
			exposure_down -= trackParams[IDX_TRACK_ACQUISITION_EXP_INCREMENT].parameter;
			counter++; 
		} else{
			searching_down = false;
		}		
	}

	// Sweep gains
	// for(gain++; gain <= 10; gain++)
	// {
	// 	camera.config->gain_dB.write(gain);
	// 	for(int i = 0; i < camera.requestQueueSize; i++)
	// 	{
	// 		if(camera.waitForFrame())
	// 		{
	// 			Image frame(camera);
	// 			camera.requestFrame();
	// 			if(verifyFrame(frame) && windowAndTune()) return true;
	// 		}
	// 	}
	// }

	return false;
}

// TODO: This should have more sophisticated checks eventually
//-----------------------------------------------------------------------------
bool Tracking::verifyFrame(Image& frame, bool printFailure)
//-----------------------------------------------------------------------------
{
	if(frame.histBrightest > trackParams[IDX_TRACK_ACQUISITION_BRIGHTNESS].parameter &&
	   frame.histBrightest > frame.histPeak &&
	   frame.histBrightest - frame.histPeak > trackParams[IDX_TRACK_GOOD_PEAKTOMAX_DISTANCE].parameter)
	{
		// All checks passed and we have some good groups!
		if(frame.performPixelGrouping(0, false) > 0)
		{
			log(pat_health_port, fileStream, "In tracking.cpp Tracking::verifyFrame - Frame verified, tuning camera parameters");
			return true;
		}
		else if(printFailure){
			log(pat_health_port, fileStream, "In tracking.cpp Tracking::verifyFrame - Frame has good properties but grouping did not succeed");
		}
	}
	else if(printFailure){
		log(pat_health_port, fileStream, "In tracking.cpp Tracking::verifyFrame - Frame check failed! ",
			"histBrightest =", frame.histBrightest, "(", trackParams[IDX_TRACK_ACQUISITION_BRIGHTNESS].parameter, ") and histPeak =", frame.histPeak);
	}
	send_packet_pat_health(pat_health_port);
	return false;
}

// Make window around brightest area, fine tune exposure/gain with no binning
//-----------------------------------------------------------------------------
bool Tracking::windowAndTune(Image& frame, Group& beacon, AOI& beaconWindow, int maxExposure)
//-----------------------------------------------------------------------------
{
	bool success = autoTuneExposure(beacon, maxExposure);
	if (success)
	{
		beaconWindow.x = camera.config->aoiStartX.read();
		beaconWindow.y = camera.config->aoiStartY.read();
		beaconWindow.w = camera.config->aoiWidth.read();
		beaconWindow.h = camera.config->aoiHeight.read();
	}
	return success;

	// // Prepare a small window around brightest group for tuning
	// double fullX = frame.groups[0].x * 2 + beaconWindow.x;
	// double fullY = frame.groups[0].y * 2 + beaconWindow.y;
	// int maxValue = frame.groups[0].valueMax;
	// double fullX_test, fullY_test;

	// for(int8_t i = 0; i < TRACK_TUNING_MAX_ATTEMPTS; i++)
	// {
	// 	camera.config->binningMode.write(cbmOff);
	// 	if(beaconWindow.w < TRACK_ACQUISITION_WINDOW){
	// 		camera.setCenteredWindow(fullX, fullY, beaconWindow.w);
	// 		log(pat_health_port, fileStream,  "In tracking.cpp Tracking::windowAndTune - Prepared windowed tuning frame at ", fullX - CAMERA_WIDTH/2, fullY - CAMERA_HEIGHT/2, " rel-to-center, w = h =", beaconWindow.w, "]");
	// 	} else{
	// 		camera.setCenteredWindow(fullX, fullY, TRACK_ACQUISITION_WINDOW);
	// 		log(pat_health_port, fileStream,  "In tracking.cpp Tracking::windowAndTune - Prepared windowed tuning frame at ", fullX - CAMERA_WIDTH/2, fullY - CAMERA_HEIGHT/2, " rel-to-center, w = h =", TRACK_ACQUISITION_WINDOW, "]");
	// 	}

	// 	// Try tuning the windowed frame
	// 	bool success = autoTuneExposure(beacon, maxExposure);

	// 	// Switch back to full frame
	// 	camera.config->binningMode.write(cbmBinningHV);
	// 	camera.setWindow(beaconWindow); //camera.setFullWindow();
	// 	camera.requestFrame();
	// 	// If passed, verify if we are on the right spot in initial frame
	// 	if(success)
	// 	{
	// 		if(camera.waitForFrame())
	// 		{
	// 			Image test(camera, fileStream, pat_health_port);
	// 			if(test.performPixelGrouping(0, false) > 0)
	// 			{
	// 				fullX_test = test.groups[0].x * 2 + beaconWindow.x;
	// 				fullY_test = test.groups[0].y * 2 + beaconWindow.y;
	// 				if(abs(fullX_test - fullX) < TRACK_TUNING_POSITION_TOLERANCE &&
	// 				   abs(fullY_test - fullY) < TRACK_TUNING_POSITION_TOLERANCE &&
	// 				   abs((int)test.groups[0].valueMax - maxValue) < TRACK_TUNING_BRIGHTNESS_TOLERANCE)
	// 				{
	// 					// Tuned spot is at a good location, success
	// 					if(beaconWindow.w < TRACK_ACQUISITION_WINDOW){
	// 						camera.setCenteredWindow(fullX, fullY, beaconWindow.w);
	// 					} else{
	// 						camera.setCenteredWindow(fullX, fullY, TRACK_ACQUISITION_WINDOW);
	// 					}
	// 					camera.config->binningMode.write(cbmOff);
	// 					// update beacon window properties
	// 					beaconWindow.x = camera.config->aoiStartX.read();
	// 					beaconWindow.y = camera.config->aoiStartY.read();
	// 					beaconWindow.w = camera.config->aoiWidth.read();
	// 					beaconWindow.h = camera.config->aoiHeight.read();
	// 					// exit:
	// 					return true;
	// 				} else{
	// 					log(pat_health_port, fileStream,  "In tracking.cpp Tracking::windowAndTune - Final test check failed: ",
	// 						"abs(fullX_test - fullX) >= TRACK_TUNING_POSITION_TOLERANCE: ", abs(fullX_test - fullX), ">=", TRACK_TUNING_POSITION_TOLERANCE, " OR ",
	// 						"abs(fullY_test - fullY) >= TRACK_TUNING_POSITION_TOLERANCE: ", abs(fullY_test - fullY), ">=", TRACK_TUNING_POSITION_TOLERANCE, " OR ",
	// 						"abs((int)test.groups[0].valueMax - maxValue) >= TRACK_TUNING_BRIGHTNESS_TOLERANCE): ", abs((int)test.groups[0].valueMax - maxValue), ">=", TRACK_TUNING_BRIGHTNESS_TOLERANCE);
							
	// 				}

	// 				// Tuned the wrong area, repeat
	// 				fullX = fullX_test;
	// 				fullY = fullY_test;
	// 				maxValue = test.groups[0].valueMax;
	// 			}
	// 			else break;
	// 		}
	// 		else break;
	// 	}
	// }
	// log(pat_health_port, fileStream, "In tracking.cpp Tracking::windowAndTune - Camera tuning failed.");
	// return false;
}

// Try auto tuning the exposure for the current spot
//-----------------------------------------------------------------------------
bool Tracking::autoTuneExposure(Group& beacon, int maxExposure)
//-----------------------------------------------------------------------------
{
	AOI tuningWindow;

	// Helper testing inline function
	auto test = [&](bool desaturating) -> bool
	{
		camera.requestFrame();
		if(camera.waitForFrame())
		{
			Image test(camera, fileStream, pat_health_port); //, beaconSmoothing
			if(test.performPixelGrouping(0, false) > 0)
			{
				Group& spot = test.groups[0];
				// Copy properties
				beacon.x = test.area.x + spot.x;
				beacon.y = test.area.y + spot.y;
				beacon.valueMax = spot.valueMax;
				beacon.valueSum = spot.valueSum;
				beacon.pixelCount = spot.pixelCount;
				if(desaturating && (spot.valueMax <= trackParams[IDX_TRACK_HAPPY_BRIGHTNESS].parameter)) return true;
				if(!desaturating && (spot.valueMax >= trackParams[IDX_TRACK_HAPPY_BRIGHTNESS].parameter)) return true;
				updateTrackingWindow(test, spot, tuningWindow);
				camera.setWindow(tuningWindow);
			}
		}
		return false;
	};

	// Grab a frame to determine the next step
	camera.requestFrame();
	if(camera.waitForFrame())
	{
		Image frame(camera, fileStream, pat_health_port);
		if(frame.performPixelGrouping(0, false) > 0)
		{
			//Determine smoothing
			//beaconSmoothing = calibration.determineSmoothing(frame);
			//frame.applyFastBlur(beaconSmoothing);
			// if(frame.performPixelGrouping() > 0){...}
			Group& spot = frame.groups[0];
			// Check if tuning is necessary
			if(abs((int)spot.valueMax - trackParams[IDX_TRACK_HAPPY_BRIGHTNESS].parameter) < trackParams[IDX_TRACK_TUNING_TOLERANCE].parameter){
				// Copy properties
				beacon.x = frame.area.x + spot.x;
				beacon.y = frame.area.y + spot.y;
				beacon.valueMax = spot.valueMax;
				beacon.valueSum = spot.valueSum;
				beacon.pixelCount = spot.pixelCount;
				log(pat_health_port, fileStream, "In tracking.cpp Tracking::autoTuneExposure - Tuning unneccessary. Exiting...");
				return true;
			}
			// Start Tuning
			updateTrackingWindow(frame, spot, tuningWindow);
			camera.setWindow(tuningWindow);
			bool desaturating;

			if(spot.valueMax > trackParams[IDX_TRACK_HAPPY_BRIGHTNESS].parameter)
			{
				desaturating = true;
				// log(pat_health_port, fileStream, "In tracking.cpp Tracking::autoTuneExposure - ",
				// "(spot.valueMax = ", spot.valueMax, ") > (TRACK_HAPPY_BRIGHTNESS = ", TRACK_HAPPY_BRIGHTNESS,"). Reducing exposure..."); 
				// Start decreasing exposure
				int exposure = camera.config->expose_us.read();
				for(exposure -= exposure/trackParams[IDX_TRACK_TUNING_EXP_DIVIDER].parameter; (exposure >= trackParams[IDX_TRACK_MIN_EXPOSURE].parameter) && (exposure/trackParams[IDX_TRACK_TUNING_EXP_DIVIDER].parameter >= 1); exposure -= exposure/trackParams[IDX_TRACK_TUNING_EXP_DIVIDER].parameter)
				{
					camera.config->expose_us.write(exposure);
					if(test(desaturating)){
						// log(pat_health_port, fileStream, "In tracking.cpp Tracking::autoTuneExposure - Exposure tuning successful. exposure = ", exposure);
						return true;
					}
				}

				// log(pat_health_port, fileStream, "In tracking.cpp Tracking::autoTuneExposure - Exposure tuning failed. Reducing gain...");
				// // Start decreasing gain if it's non-zero
				// int gain = camera.config->gain_dB.read();
				// for(gain--; gain > 0; gain--)
				// {
				// 	camera.config->gain_dB.write(gain);
				// 	if(test(desaturating)){
				// 		log(pat_health_port, fileStream, "In tracking.cpp Tracking::autoTuneExposure - Gain tuning successful. gain = ", gain);
				// 		return true;
				// 	}
				// }

				// Camera reached lower limit, too high power
				log(pat_health_port, fileStream, "In tracking.cpp Tracking::autoTuneExposure - Unable to reduce brightness to desired level (TRACK_HAPPY_BRIGHTNESS = ", trackParams[IDX_TRACK_HAPPY_BRIGHTNESS].parameter, ") with minimum parameters: TRACK_MIN_EXPOSURE = ", trackParams[IDX_TRACK_MIN_EXPOSURE].parameter, ", gain = 0");
			}
			// Otherwise, have to increase exposure
			else
			{
				desaturating = false; 
				// log(pat_health_port, fileStream, "In tracking.cpp Tracking::autoTuneExposure - ",
				// "(spot.valueMax = ", spot.valueMax, ") <= (TRACK_HAPPY_BRIGHTNESS = ", TRACK_HAPPY_BRIGHTNESS,"). Increasing exposure...");
				// Start increasing exposure
				int exposure = camera.config->expose_us.read();
				for(exposure += exposure/trackParams[IDX_TRACK_TUNING_EXP_DIVIDER].parameter; (exposure <= maxExposure); exposure += exposure/trackParams[IDX_TRACK_TUNING_EXP_DIVIDER].parameter)
				{
					camera.config->expose_us.write(exposure);
					if(test(desaturating)){
						// log(pat_health_port, fileStream, "In tracking.cpp Tracking::autoTuneExposure - Exposure tuning successful. exposure = ", exposure);
						return true;
					}
				}

				// Start increasing gain
				// int gain = camera.config->gain_dB.read();
				// for(gain++; gain <= TRACK_MAX_GAIN; gain++)
				// {
				// 	camera.config->gain_dB.write(gain);
				// 	if(test(desaturating)){
				// 		log(pat_health_port, fileStream, "In tracking.cpp Tracking::autoTuneExposure - Gain tuning successful. gain = ", gain);
				// 		return true;
				// 	}
				// }

				// Very high parameters reached
				log(pat_health_port, fileStream, "In tracking.cpp Tracking::autoTuneExposure - Unable to increase brightness to desired level (TRACK_HAPPY_BRIGHTNESS = ", trackParams[IDX_TRACK_HAPPY_BRIGHTNESS].parameter, ") with maximum parameters: maxExposure = ", maxExposure, ", TRACK_MAX_GAIN = ", TRACK_MAX_GAIN);
			}
		}
	}
	// log(pat_health_port, fileStream, "In tracking.cpp Tracking::autoTuneExposure - Exposure auto tuning failed!");
	return false;
}

// Update camera window based on spot's size and its location in window
//-----------------------------------------------------------------------------
void Tracking::updateTrackingWindow(Image& frame, Group& spot, AOI& window)
//-----------------------------------------------------------------------------
{
	float spotWidth = sqrt(1.5f * (float)spot.pixelCount);
	float distanceLimit = spotWidth > trackParams[IDX_TRACK_MIN_SPOT_LIMIT].parameter ? spotWidth : trackParams[IDX_TRACK_MIN_SPOT_LIMIT].parameter;

	// Check maximum allowed width discrepancy due to quantization & tolerance
	// Then check if location of centroid is in good bounds
	if(abs(window.w - (spotWidth + 2*trackParams[IDX_TRACK_MIN_SPOT_LIMIT].parameter)) > (30 + trackParams[IDX_TRACK_WINDOW_SIZE_TOLERANCE].parameter) ||
	   abs(window.h - (spotWidth + 2*trackParams[IDX_TRACK_MIN_SPOT_LIMIT].parameter)) > (30 + trackParams[IDX_TRACK_WINDOW_SIZE_TOLERANCE].parameter) ||
	   spot.x < distanceLimit || spot.x > window.w - distanceLimit ||
	   spot.y < distanceLimit || spot.y > window.h - distanceLimit)
	{
		// Update is most likely needed, calculate new bounds
		int width = spotWidth + 2*trackParams[IDX_TRACK_MIN_SPOT_LIMIT].parameter, height = width, temp;
		int x = frame.area.x + spot.x - width/2;
		int y = frame.area.y + spot.y - height/2;

		// Canvas clamping and quantization
		// x and width must be a multiple of 16
		if(x >= 0)
		{
			temp = x % 16;
			x -= temp;
			width += temp;
		}
		else
		{
			width += x;
			x = 0;
		}
		if(width % 16 != 0) width += (16 - (width % 16));

		// y and height must be a multiple of 8
		if(y >= 0)
		{
			temp = y % 8;
			y -= temp;
			height += temp;
		}
		else
		{
			height += y;
			y = 0;
		}
		if(height % 8 != 0) height += (8 - (height % 8));

		// Final check
		if(x != window.x || y != window.y || height != window.h || width != window.w)
		{
			// log(pat_health_port, fileStream, "In tracking.cpp Tracking::updateTrackingWindow - Updated window to", width, "x", height, " at ", x - CAMERA_WIDTH/2, y - CAMERA_HEIGHT/2, " rel-to-center]");
			window.x = x;
			window.y = y;
			window.w = width;
			window.h = height;
		}
	}
}

// Finds group that is most likely the spot we are looking for, based on its last properties
//-----------------------------------------------------------------------------
int Tracking::findSpotCandidate(Image& frame, Group& oldSpot, double *difference)
//-----------------------------------------------------------------------------
{
	int candidate = 0;
	double minDifference = 1e10;

	for(unsigned int i = 0; i < frame.groups.size(); i++)
	{
		double difference;

		// If size uncertain just check position
		if(oldSpot.pixelCount == 0)
		{
			difference = abs(frame.area.x + frame.groups[i].x - oldSpot.x) +
				abs(frame.area.y + frame.groups[i].y - oldSpot.y);
		}
		else
		{
			// Sum size, pixel count and brightness (half weight) offsets
			difference = abs(frame.area.x + frame.groups[i].x - oldSpot.x) +
				abs(frame.area.y + frame.groups[i].y - oldSpot.y) +
				abs((int)frame.groups[i].pixelCount - (int)oldSpot.pixelCount) +
				abs((int)frame.groups[i].valueMax/2 - (int)oldSpot.valueMax/2);
		}

		if(difference < minDifference)
		{
			minDifference = difference;
			candidate = i;
		}
	}

	*difference = minDifference;

	return candidate;
}

// Control FSM in open loop using calibrated projections
//-----------------------------------------------------------------------------
void Tracking::controlOpenLoop(FSM& fsm, double x, double y)
//-----------------------------------------------------------------------------
{
	actionX = calibration.affineTransformX(x, y);
	actionY = calibration.affineTransformY(x, y);
	fsm.setNormalizedAngles(actionX, actionY);
	lastUpdate = steady_clock::now();
	// log(pat_health_port, fileStream, "In Tracking::controlOpenLoop - Updating FSM position to: ",
	// "x_pxls = ", x, " -> x_normalized = ", actionX, ". ", "y_pxls = ", y, " -> y_normalized = ", actionY, ". ");
	//if((actionX > 0.3) || (actionX < -0.3)){log(pat_health_port, fileStream, "In Tracking::controlOpenLoop - Warning: (|actionX| = |", actionX, "|) > 0.3");}
	//if((actionY > 0.3) || (actionY < -0.3)){log(pat_health_port, fileStream, "In Tracking::controlOpenLoop - Warning: (|actionY| = |", actionY, "|) > 0.3");}
}

// Control FSM with feedback to setpoint using integral control
//-----------------------------------------------------------------------------
void Tracking::control(FSM& fsm, double x, double y, double spX, double spY)
//-----------------------------------------------------------------------------
{
	// Calculate elapsed time
	time_point<steady_clock> now = steady_clock::now();
	duration<double> diff = now - lastUpdate;
	double Ts = diff.count() > track_control_max_ts ? track_control_max_ts : diff.count();

	// log(pat_health_port, fileStream, "Ts is", diff.count());

	// Calculate errors on FSM
	double ex = calibration.transformDx(spX - x, spY - y);
	double ey = calibration.transformDy(spX - x, spY - y);

	// Calculate integral deltas
	double dxFSM = ex * Ts * trackParams[IDX_TRACK_CONTROL_I].parameter;
	double dyFSM = ey * Ts * trackParams[IDX_TRACK_CONTROL_I].parameter;
	if(dxFSM > CALIB_FSM_MAX_DELTA) dxFSM = CALIB_FSM_MAX_DELTA;
	if(dyFSM > CALIB_FSM_MAX_DELTA) dyFSM = CALIB_FSM_MAX_DELTA;

	// Anti-windup
	if(abs(actionX + dxFSM) > 1)
	{
		if(actionX + dxFSM > 1) actionX = 1;
		else if(actionX + dxFSM < -1) actionX = -1;
	}
	else actionX += dxFSM;

	if(abs(actionY + dyFSM) > 1)
	{
		if(actionY + dyFSM > 1) actionY = 1;
		else if(actionY + dyFSM < -1) actionY = -1;
	}
	else actionY += dyFSM;

	// Update output
	fsm.setNormalizedAngles(actionX, actionY);
	lastUpdate = now;
	// log(pat_health_port, fileStream, "In Tracking::control - Updating FSM position: ",
	// "x_current = ", x, " -> x_setpoint = ", spX, " -> x_normalized = ", actionX, ". ", 
	// "y_current = ", y, " -> y_setpoint = ", spY, " -> y_normalized = ", actionY, ". ");
	//if((actionX > 0.3) || (actionX < -0.3)){log(pat_health_port, fileStream, "In Tracking::controlOpenLoop - Warning: (|actionX| = |", actionX, "|) > 0.3");}
	//if((actionY > 0.3) || (actionY < -0.3)){log(pat_health_port, fileStream, "In Tracking::controlOpenLoop - Warning: (|actionY| = |", actionY, "|) > 0.3");}
}

// Check whether spots are at a safe distance and closed-loop tracking is possible
//-----------------------------------------------------------------------------
bool Tracking::distanceIsSafe(Group& beacon, Group& calib, bool openloop)
//-----------------------------------------------------------------------------
{
	double distance = sqrt((beacon.x - calib.x)*(beacon.x - calib.x) +
		(beacon.y - calib.y)*(beacon.y - calib.y));
	if(openloop && distance > trackParams[IDX_TRACK_SAFE_DISTANCE_ALLOW].parameter) return true;
	if(!openloop && distance > trackParams[IDX_TRACK_SAFE_DISTANCE_PANIC].parameter) return true;
	return false;
}

//Exposure control function: checks brightness of beacon spot and changes exposure to compensate if necessary, pg
//-----------------------------------------------------------------------------
int Tracking::controlExposure(int valueMax, int exposure, int maxBcnExposure)
//-----------------------------------------------------------------------------
{
	int brightnessDifference = valueMax - trackParams[IDX_TRACK_HAPPY_BRIGHTNESS].parameter; //difference in brightness from ideal

	if(abs(brightnessDifference) > trackParams[IDX_TRACK_EXP_CONTROL_TOLERANCE].parameter) //outside acceptable brightness
	{
		if(brightnessDifference > 0) //too bright, decrease exposure
		{
			int newExposure = exposure - exposure/trackParams[IDX_TRACK_EXP_CONTROL_DIVIDER].parameter; //use divider defined in header
			if(newExposure > trackParams[IDX_TRACK_MIN_EXPOSURE].parameter){exposure = newExposure;} //limit at min exposure
			else{exposure = trackParams[IDX_TRACK_MIN_EXPOSURE].parameter;} //set to limit if necessary
		}
		else //too dim, increase exposure
		{
			int newExposure = exposure + exposure/trackParams[IDX_TRACK_EXP_CONTROL_DIVIDER].parameter; //use divider defined in header
			if(newExposure < maxBcnExposure){exposure = newExposure;} //limit at max exposure
			else{exposure = maxBcnExposure;} //set to limit if necessary
		}

		// log(pat_health_port, fileStream, "In Tracking::controlExposure - Adjusting beacon exposure to: ", exposure, " b/c ",
		// "(abs(brightnessDifference) = ", abs(brightnessDifference), ") > (TRACK_EXP_CONTROL_TOLERANCE = ", TRACK_EXP_CONTROL_TOLERANCE, "). ");
	}
	//else{
	//	log(pat_health_port, fileStream, "In Tracking::controlExposure - Brightness difference within tolerance: ",
	//	"(abs(brightnessDifference) = ", abs(brightnessDifference), ") <= (TRACK_EXP_CONTROL_TOLERANCE = ", TRACK_EXP_CONTROL_TOLERANCE, "). ");
	//}

	return exposure;
}
