// NODE FSM Calibration & Calibration laser algorithms
// Authors: Ondrej Cierny, Peter Grenfell
#include "calibration.h"
#include "tracking.h"
#include <chrono>
#include <thread>
#include <cmath>

//Constructor
//-----------------------------------------------------------------------------
Calibration::Calibration(Camera& camera, FSM& fsm, std::ofstream &fileStreamIn, zmq::socket_t &pat_health_port_in):
camera(camera), fsm(fsm), fileStream(fileStreamIn), pat_health_port(pat_health_port_in)
//-----------------------------------------------------------------------------
{
	if(!getCalibParams()){
		//assign default parameters if CSV parameter retrieval fails
		log(pat_health_port, fileStream, "In calibration.cpp - Calibration: using default parameters:");
		
		//Set names
		calibParams[IDX_CALIB_BIG_WINDOW].name = "CALIB_BIG_WINDOW";
		calibParams[IDX_CALIB_SMALL_WINDOW].name = "CALIB_SMALL_WINDOW";
		calibParams[IDX_CALIB_MIN_BRIGHTNESS].name = "CALIB_MIN_BRIGHTNESS";
		calibParams[IDX_CALIB_MAX_EXPOSURE].name = "CALIB_MAX_EXPOSURE";
		calibParams[IDX_CALIB_MIN_EXPOSURE].name = "CALIB_MIN_EXPOSURE";
		calibParams[IDX_CALIB_EXP_INCREMENT].name = "CALIB_EXP_INCREMENT";
		calibParams[IDX_CALIB_MAX_GAIN].name = "CALIB_MAX_GAIN";
		calibParams[IDX_CALIB_EXP_DIVIDER].name = "CALIB_EXP_DIVIDER";
		calibParams[IDX_CALIB_MAX_SMOOTHING].name = "CALIB_MAX_SMOOTHING";
		calibParams[IDX_CALIB_GOOD_PEAKTOMAX_DISTANCE].name = "CALIB_GOOD_PEAKTOMAX_DISTANCE";
		calibParams[IDX_CALIB_HAPPY_BRIGHTNESS].name = "CALIB_HAPPY_BRIGHTNESS";
		calibParams[IDX_CALIB_TUNING_TOLERANCE].name = "CALIB_TUNING_TOLERANCE";
		calibParams[IDX_CALIB_FSM_DISPLACEMENT_TOL].name = "CALIB_FSM_DISPLACEMENT_TOL";

		//Set default parameters
		calibParams[IDX_CALIB_BIG_WINDOW].parameter = CALIB_BIG_WINDOW; //implemented
		calibParams[IDX_CALIB_SMALL_WINDOW].parameter = CALIB_SMALL_WINDOW; //implemented
		calibParams[IDX_CALIB_MIN_BRIGHTNESS].parameter = CALIB_MIN_BRIGHTNESS; //implemented
		calibParams[IDX_CALIB_MAX_EXPOSURE].parameter = CALIB_MAX_EXPOSURE; //implemented
		calibParams[IDX_CALIB_MIN_EXPOSURE].parameter = CALIB_MIN_EXPOSURE; //implemented
		calibParams[IDX_CALIB_EXP_INCREMENT].parameter = CALIB_EXP_INCREMENT; //implemented
		calibParams[IDX_CALIB_MAX_GAIN].parameter = CALIB_MAX_GAIN; //implemented
		calibParams[IDX_CALIB_EXP_DIVIDER].parameter = CALIB_EXP_DIVIDER; //implemented
		calibParams[IDX_CALIB_MAX_SMOOTHING].parameter = CALIB_MAX_SMOOTHING; //implemented
		calibParams[IDX_CALIB_GOOD_PEAKTOMAX_DISTANCE].parameter = CALIB_GOOD_PEAKTOMAX_DISTANCE; //implemented
		calibParams[IDX_CALIB_HAPPY_BRIGHTNESS].parameter = CALIB_HAPPY_BRIGHTNESS; //implemented
		calibParams[IDX_CALIB_TUNING_TOLERANCE].parameter = CALIB_TUNING_TOLERANCE; //implemented
		calibParams[IDX_CALIB_FSM_DISPLACEMENT_TOL].parameter = CALIB_FSM_DISPLACEMENT_TOL; //implemented

		//Display parameters
		for (size_t i = 0; i < NUM_CALIB_PARAMS; i++)
		{
			log(pat_health_port, fileStream, "In calibration.cpp - Calibration: ", calibParams[i].name, ": ", calibParams[i].parameter);
		}			
	}
};

//Load modifiable constant parameters from external CSV file
//-----------------------------------------------------------------------------
bool Calibration::getCalibParams()
//-----------------------------------------------------------------------------
{
	calibParamStruct calibParam; //temp offsetParamStruct for use in the while loop
    ifstream inFile("/root/lib/calibParams.csv"); //our file
    string line;
    int linenum = 0;
	log(pat_health_port, fileStream, "In calibration.cpp - getCalibParams: Retrieving Calibration Parameters from /root/lib/calibParams.csv");
    if(inFile.is_open()){
		while (getline (inFile, line))
		{
			istringstream linestream(line);
			string item;
			//use this to get up to the first comma
			getline(linestream, item, ',');
			calibParam.name = item;
			//convert to a string stream and then put in id.
			getline(linestream, item, ',');
			stringstream ss(item);
			ss >> calibParam.parameter;
			//report read data
			log(pat_health_port, fileStream, "In calibration.cpp - getCalibParams: ", calibParam.name, ": ", calibParam.parameter);
			//add the new data to the list
			calibParams[linenum] = calibParam;
			linenum++;
		}
		if(linenum != NUM_CALIB_PARAMS){
			log(pat_health_port, fileStream, "In calibration.cpp - getCalibParams: /root/lib/calibParams.csv is missing data: ", linenum, " lines found out of ", NUM_CALIB_PARAMS);
            return false;
        } else{
			return true;
		}
	} else{
		log(pat_health_port, fileStream, "In calibration.cpp - getCalibParams: /root/lib/calibParams.csv did not open or doesn't exist.");
		return false;
	}
}

// Find range of exposure settings where calibration spot has good parameters
// Also find the gains (function) associated with this range
// Also save properties of spot for re-use in main routine
//-----------------------------------------------------------------------------
bool Calibration::findExposureRange(bool testLaser, std::string filePath)
//-----------------------------------------------------------------------------
{
	//int skip = camera.queuedCount;
	int exposure = calibParams[IDX_CALIB_MIN_EXPOSURE].parameter; 
	int gain = 0;
	//preferredExpo = CALIB_MAX_EXPOSURE + 1;
	//lowestExpoNoGain = 0;

	// Configure camera for big window around center
	//camera.config->gain_dB.write(gain);
	//camera.config->binningMode.write(cbmOff);
	camera.setCenteredWindow(CAMERA_WIDTH/2, CAMERA_HEIGHT/2, calibParams[IDX_CALIB_BIG_WINDOW].parameter);
    
	//Try with minimum exposure
	camera.config->expose_us.write(exposure);
    camera.requestFrame();	
	//camera.ignoreNextFrames(skip); // Skip pre-queued old frames
	if(camera.waitForFrame())
	{
		Image init(camera, fileStream, pat_health_port);
		if(verifyFrame(init) && windowAndTune(init, testLaser)){return true;}
	}

	//Start incrementing upwards
	for(; exposure <= calibParams[IDX_CALIB_MAX_EXPOSURE].parameter; exposure += calibParams[IDX_CALIB_EXP_INCREMENT].parameter){
		camera.config->expose_us.write(exposure);
		camera.requestFrame();	
		if(camera.waitForFrame())
		{
			Image frame(camera, fileStream, pat_health_port);
			if(verifyFrame(frame) && windowAndTune(frame, testLaser)){return true;}
		}
	}
	return false; 
}

// Verify calibration spot in frame prior to exposure tuning
//-----------------------------------------------------------------------------
bool Calibration::verifyFrame(Image& frame)
//-----------------------------------------------------------------------------
{
	if(frame.histBrightest > calibParams[IDX_CALIB_MIN_BRIGHTNESS].parameter &&
	   frame.histBrightest > frame.histPeak &&
	   frame.histBrightest - frame.histPeak > calibParams[IDX_CALIB_GOOD_PEAKTOMAX_DISTANCE].parameter)
	{
		// All checks passed and we have some good groups!
		if(frame.performPixelGrouping() > 0)
		{
			preferredExpo = camera.config->expose_us.read();
			log(pat_health_port, fileStream, "In calibration.cpp Calibration::verifyFrame - Frame verified at exp = ", preferredExpo, ". Tuning camera parameters.");
			return true;
		}
		else log(pat_health_port, fileStream, "In calibration.cpp Calibration::verifyFrame - Frame has good properties but grouping did not succeed");
	}
	else{
		log(pat_health_port, fileStream, "In calibration.cpp Calibration::verifyFrame - Frame property check failed! ",
			"frame.histBrightest = ", frame.histBrightest, " should be >= CALIB_MIN_BRIGHTNESS = ", calibParams[IDX_CALIB_MIN_BRIGHTNESS].parameter,
			", \n and frame.histBrightest = ", frame.histBrightest, " should be > frame.histPeak = ", frame.histPeak,
			", \n and (frame.histBrightest - frame.histPeak) = ", frame.histBrightest - frame.histPeak, " should be > CALIB_GOOD_PEAKTOMAX_DISTANCE = ", calibParams[IDX_CALIB_GOOD_PEAKTOMAX_DISTANCE].parameter);
	}
	return false;
}

// Tune exposure for tracking frame
//-----------------------------------------------------------------------------
bool Calibration::windowAndTune(Image& frame, bool testLaser)
//-----------------------------------------------------------------------------
{
	// Helper testing inline function
	auto test = [&](bool desaturating) -> bool
	{
		camera.requestFrame();
		if(camera.waitForFrame())
		{
			Image test(camera, fileStream, pat_health_port);
			if(test.performPixelGrouping(0, false) > 0)
			{
				Group& spot = test.groups[0];
				if(desaturating && (spot.valueMax <= calibParams[IDX_CALIB_HAPPY_BRIGHTNESS].parameter)) return true;
				if(!desaturating && (spot.valueMax >= calibParams[IDX_CALIB_HAPPY_BRIGHTNESS].parameter)) return true;
			}
		}
		return false;
	};

	// Check initial image values
	Group& spot = frame.groups[0];

	// Set small window on spot location and search for range
	camera.config->expose_us.write(preferredExpo);
	if(testLaser){
		camera.setCenteredWindow(frame.area.x + spot.x, frame.area.y + spot.y, calibParams[IDX_CALIB_SMALL_WINDOW].parameter);
		log(pat_health_port, fileStream, "In calibration.cpp Calibration::windowAndTune - Tuning calib exposure, starting with expo = ", preferredExpo, "us at [",
			frame.area.x + spot.x, ",", frame.area.y + spot.y, ", window size: ", calibParams[IDX_CALIB_SMALL_WINDOW].parameter ,"]"); //with smoothing", smoothing);
	} else{
		log(pat_health_port, fileStream, "In calibration.cpp Calibration::windowAndTune - Tuning calib exposure, starting with expo = ", preferredExpo, "us at [",
			CAMERA_WIDTH/2, ",", CAMERA_HEIGHT/2, ", window size: ", calibParams[IDX_CALIB_BIG_WINDOW].parameter ,"]"); //with smoothing", smoothing);
	}

	// Grab a frame to determine the next step
	camera.requestFrame();
	if(camera.waitForFrame())
	{
		Image frame(camera, fileStream, pat_health_port);
		if(frame.performPixelGrouping() > 0)
		{
			Group& spot = frame.groups[0];
			// Check if tuning is necessary
			if(abs((int)spot.valueMax - calibParams[IDX_CALIB_HAPPY_BRIGHTNESS].parameter) < calibParams[IDX_CALIB_TUNING_TOLERANCE].parameter){
				log(pat_health_port, fileStream, "In calibration.cpp Calibration::windowAndTune - Tuning unneccessary. Exiting...");
				return true;
			}
			// Start Tuning
			bool desaturating;
			if(spot.valueMax > calibParams[IDX_CALIB_HAPPY_BRIGHTNESS].parameter)
			{
				desaturating = true;
				log(pat_health_port, fileStream, "In calibration.cpp Calibration::windowAndTune - ",
				"(spot.valueMax = ", spot.valueMax, ") > (CALIB_HAPPY_BRIGHTNESS = ", calibParams[IDX_CALIB_HAPPY_BRIGHTNESS].parameter,"). Reducing exposure..."); 
				// Start decreasing exposure
				int exposure = preferredExpo;
				for(exposure -= exposure/calibParams[IDX_CALIB_EXP_DIVIDER].parameter; (exposure >= calibParams[IDX_CALIB_MIN_EXPOSURE].parameter) && (exposure/calibParams[IDX_CALIB_EXP_DIVIDER].parameter >= 1); exposure -= exposure/calibParams[IDX_CALIB_EXP_DIVIDER].parameter)
				{
					camera.config->expose_us.write(exposure);
					if(test(desaturating)){
						log(pat_health_port, fileStream, "In calibration.cpp Calibration::windowAndTune - Exposure tuning successful. exposure = ", exposure);
						preferredExpo = exposure;
						return true;
					}
				}

				// log(pat_health_port, fileStream, "In calibration.cpp Calibration::windowAndTune - Exposure tuning failed. Reducing gain...");
				// // Start decreasing gain if it's non-zero
				// int gain = camera.config->gain_dB.read();
				// for(gain--; gain > 0; gain--)
				// {
				// 	camera.config->gain_dB.write(gain);
				// 	if(test(desaturating)){
				// 		log(pat_health_port, fileStream, "In tracking.cpp calibration.cpp Calibration::windowAndTune - Gain tuning successful. gain = ", gain);
				// 		return true;
				// 	}
				// }

				// Camera reached lower limit, too high power
				log(pat_health_port, fileStream, "In calibration.cpp Calibration::windowAndTune - Unable to reduce brightness to desired level (CALIB_HAPPY_BRIGHTNESS = ", calibParams[IDX_CALIB_HAPPY_BRIGHTNESS].parameter, "). Using minimum parameters: CALIB_MIN_EXPOSURE = ", calibParams[IDX_CALIB_MIN_EXPOSURE].parameter, ", gain = 0");
				preferredExpo = calibParams[IDX_CALIB_MIN_EXPOSURE].parameter;
				camera.config->expose_us.write(preferredExpo);
				return true;
			}
			// Otherwise, have to increase exposure
			else
			{
				desaturating = false; 
				log(pat_health_port, fileStream, "In calibration.cpp Calibration::windowAndTune - ",
				"(spot.valueMax = ", spot.valueMax, ") <= (CALIB_HAPPY_BRIGHTNESS = ", calibParams[IDX_CALIB_HAPPY_BRIGHTNESS].parameter,"). Increasing exposure...");
				// Start increasing exposure
				int exposure = preferredExpo;
				for(exposure += exposure/calibParams[IDX_CALIB_EXP_DIVIDER].parameter; (exposure <= calibParams[IDX_CALIB_MAX_EXPOSURE].parameter); exposure += exposure/calibParams[IDX_CALIB_EXP_DIVIDER].parameter)
				{
					camera.config->expose_us.write(exposure);
					if(test(desaturating)){
						log(pat_health_port, fileStream, "In calibration.cpp Calibration::windowAndTune - Exposure tuning successful. exposure = ", exposure);
						preferredExpo = exposure;
						return true;
					}
				}

				// // Start increasing gain
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
				log(pat_health_port, fileStream, "In calibration.cpp Calibration::windowAndTune - Unable to increase brightness to desired level (CALIB_HAPPY_BRIGHTNESS = ", calibParams[IDX_CALIB_HAPPY_BRIGHTNESS].parameter, "). Using maximum parameters: CALIB_MAX_EXPOSURE = ", calibParams[IDX_CALIB_MAX_EXPOSURE].parameter, ", gain = 0");
				preferredExpo = calibParams[IDX_CALIB_MAX_EXPOSURE].parameter;
				camera.config->expose_us.write(preferredExpo);
				return true;
			}
		} else{
			log(pat_health_port, fileStream, "In calibration.cpp Calibration::windowAndTune - Pixel grouping failed!");
		}
	} else{
		log(pat_health_port, fileStream, "In calibration.cpp Calibration::windowAndTune - wait for frame failed!");
	}
	log(pat_health_port, fileStream, "In calibration.cpp Calibration::windowAndTune - Exposure tuning failed!");
	return false;
}

// Finds acceptable camera settings for calib laser
// Performs a spiral calibration pattern on detector with FSM
// Collects 100 point pairs and calculates Detector -> FSM transformation parameters
//-----------------------------------------------------------------------------
bool Calibration::run(Group& calib, std::string filePath)
//-----------------------------------------------------------------------------
{
	std::vector<Pair> points;
	double omega = 2 * M_PI * 0.05f;

	// Reset FSM to center
	fsm.setNormalizedAngles(0, 0);
	this_thread::sleep_for(chrono::milliseconds(CALIB_FSM_RISE_TIME));

	// Find the exposure setting range
	//smoothing = 0;
	bool success = findExposureRange();
	//if(smoothing != 0) success = findExposureRange();

	if(success)
	{
		// Use preferred exposure
		camera.config->gain_dB.write(0);
		camera.config->expose_us.write(preferredExpo);
		for(int i = 0; i < 100; i++)
		{
			// Spiral outwards with 100 points from 0 to defined max FSM range
			double x = i * CALIB_FSM_MAX * cos(omega*i);
			double y = i * CALIB_FSM_MAX * sin(omega*i);
			fsm.setNormalizedAngles(x, y);
			this_thread::sleep_for(chrono::milliseconds(CALIB_FSM_RISE_TIME));

			camera.requestFrame();
			if(camera.waitForFrame())
			{
				Image frame(camera, fileStream, pat_health_port);

				if(frame.performPixelGrouping() > 0)
				{
					Group& spot = frame.groups[0];
					points.emplace_back(frame.area.x + spot.x, frame.area.y + spot.y, x, y);
					if(i == 0){
						//set center offset
						centerOffsetX = (frame.area.x + spot.x) - CAMERA_WIDTH/2;
						centerOffsetY = (frame.area.y + spot.y) - CAMERA_HEIGHT/2;
						log(pat_health_port, fileStream, "In calibration.cpp Calibration::run - ",
						"centerOffsetX = ", centerOffsetX, ", centerOffsetY = ", centerOffsetY);
						//copy spot properties to calib
						calib.x = frame.area.x + spot.x;
						calib.y = frame.area.y + spot.y;
						calib.valueMax = spot.valueMax;
						calib.valueSum = spot.valueSum;
						calib.pixelCount = spot.pixelCount;
						//save image telemetry
						std::string nameTag = std::string("CALIBRATION");
						std::string imageFileName = filePath + timeStamp() + std::string("_") + nameTag + std::string("_exp_") + std::to_string(camera.config->expose_us.read()) + std::string(".png");
						log(pat_health_port, fileStream, "In calibration.cpp Calibration::run - Saving image telemetry as: ", imageFileName);
						frame.savePNG(imageFileName);
					}
					if(i % 5 == 0){
						log(pat_health_port, fileStream, "In calibration.cpp Calibration::run - Pair", i, "[", frame.area.x + spot.x, ",", frame.area.y + spot.y, "] for FSM [", x, ",", y, "]");
					}

					// Move window to new location
					camera.setCenteredWindow(frame.area.x + spot.x, frame.area.y + spot.y, calibParams[IDX_CALIB_SMALL_WINDOW].parameter);
				}
				else
				{
					log(pat_health_port, fileStream, "In calibration.cpp Calibration::run - Lost spot during FSM calibration!");
				}
			}
		}

		// Reset FSM & Camera
		fsm.setNormalizedAngles(0, 0);

		//Reset Camera
		camera.setFullWindow();	

		// Check pair count
		if(points.size() < 50) return false;

		// Use pairs to calculate transform parameters
		calculateSensitivityMatrix(points);
		calculateAffineParameters(points);
		log(pat_health_port, fileStream, "In calibration.cpp Calibration::run - Calculated sensitivity matrix [", s00, s01, ";", s10, s11, "]");
		log(pat_health_port, fileStream, "In calibration.cpp Calibration::run - Calculated affine transform [", a00, a01, ";", a10, a11, "] + [", t0, ";", t1, "]");
		return true;
	}
	log(pat_health_port, fileStream, "In calibration.cpp Calibration::run - findExposureRange Failed");
	return false;
}

// Calculates affine transform parameters from Detector to FSM 2D system
// Uses least squares method with any number of point pairs > 3
// Derivation: http://www.sci.utah.edu/~gerig/CS6640-F2012/Materials/Lecture_10_18_Transformations_Durrleman.pdf
//-----------------------------------------------------------------------------
void Calibration::calculateAffineParameters(std::vector<Pair>& data)
//-----------------------------------------------------------------------------
{
	// Calculate centers of mass
	Pair centers(0,0,0,0);

	for(Pair& p : data)
	{
		centers.x0 += p.x0;
		centers.y0 += p.y0;
		centers.x1 += p.x1;
		centers.y1 += p.y1;
	}

	centers.x0 /= data.size();
	centers.y0 /= data.size();
	centers.x1 /= data.size();
	centers.y1 /= data.size();

	// Make all points centered
	for(Pair& p : data)
	{
		p.x0 -= centers.x0;
		p.y0 -= centers.y0;
		p.x1 -= centers.x1;
		p.y1 -= centers.y1;
	}

	// Matrices needed to calculate 2x2 affine matrix
	double YXSum[4] = {0,0,0,0}, XXSum[4] = {0,0,0,0};

	for(Pair& p : data)
	{
		YXSum[0] += p.x1 * p.x0;
		YXSum[1] += p.x1 * p.y0;
		YXSum[2] += p.y1 * p.x0;
		YXSum[3] += p.y1 * p.y0;
		XXSum[0] += p.x0 * p.x0;
		XXSum[1] += p.x0 * p.y0;
		XXSum[2] += p.y0 * p.x0;
		XXSum[3] += p.y0 * p.y0;
	}

	// Calculate inverse of XXSum matrix
	double d = XXSum[0]*XXSum[3] - XXSum[1]*XXSum[2];
	double XXSumInv[4] = {XXSum[3]/d, -XXSum[1]/d, -XXSum[2]/d, XXSum[0]/d};

	// Calculate affine transform parameters
	a00 = YXSum[0]*XXSumInv[0] + YXSum[1]*XXSumInv[2];
	a01 = YXSum[0]*XXSumInv[1] + YXSum[1]*XXSumInv[3];
	a10 = YXSum[2]*XXSumInv[0] + YXSum[3]*XXSumInv[2];
	a11 = YXSum[2]*XXSumInv[1] + YXSum[3]*XXSumInv[3];
	t0 = centers.x1 - a00*centers.x0 - a01*centers.y0;
	t1 = centers.y1 - a10*centers.x0 - a11*centers.y0;
}

// Calculates sensitivity matrix -> delta X/Y on Detector to delta X/Y on FSM
// Uses same least squares method as above with all measurements
//-----------------------------------------------------------------------------
void Calibration::calculateSensitivityMatrix(std::vector<Pair>& data)
//-----------------------------------------------------------------------------
{
	std::vector<Pair> deltas;

	// Create vector of deltas between first measurements around center
	for(unsigned int i = 1; i < data.size()/4; i++)
	{
		deltas.push_back(Pair(data[i].x0 - data[i-1].x0, data[i].y0 - data[i-1].y0,
			data[i].x1 - data[i-1].x1, data[i].y1 - data[i-1].y1));
	}

	// Matrices needed to calculate 2x2 sensitivity matrix
	double YXSum[4] = {0,0,0,0}, XXSum[4] = {0,0,0,0};

	for(Pair& p : deltas)
	{
		YXSum[0] += p.x1 * p.x0;
		YXSum[1] += p.x1 * p.y0;
		YXSum[2] += p.y1 * p.x0;
		YXSum[3] += p.y1 * p.y0;
		XXSum[0] += p.x0 * p.x0;
		XXSum[1] += p.x0 * p.y0;
		XXSum[2] += p.y0 * p.x0;
		XXSum[3] += p.y0 * p.y0;
	}

	// Calculate inverse of XXSum matrix
	double d = XXSum[0]*XXSum[3] - XXSum[1]*XXSum[2];
	double XXSumInv[4] = {XXSum[3]/d, -XXSum[1]/d, -XXSum[2]/d, XXSum[0]/d};

	// Calculate sensitivity matrix
	s00 = YXSum[0]*XXSumInv[0] + YXSum[1]*XXSumInv[2];
	s01 = YXSum[0]*XXSumInv[1] + YXSum[1]*XXSumInv[3];
	s10 = YXSum[2]*XXSumInv[0] + YXSum[3]*XXSumInv[2];
	s11 = YXSum[2]*XXSumInv[1] + YXSum[3]*XXSumInv[3];
}

//-----------------------------------------------------------------------------
double Calibration::affineTransformX(double x, double y)
//-----------------------------------------------------------------------------
{
	return a00*x + a01*y + t0;
}

//-----------------------------------------------------------------------------
double Calibration::affineTransformY(double x, double y)
//-----------------------------------------------------------------------------
{
	return a10*x + a11*y + t1;
}

//-----------------------------------------------------------------------------
double Calibration::transformDx(double x, double y)
//-----------------------------------------------------------------------------
{
	return s00*x + s01*y;
}

//-----------------------------------------------------------------------------
double Calibration::transformDy(double x, double y)
//-----------------------------------------------------------------------------
{
	return s10*x + s11*y;
}

// Returns gain for calibration laser for exposure time based on range search
// Just a linear interpolation between measured gains
//-----------------------------------------------------------------------------
int Calibration::gainForExposure(int exposure)
//-----------------------------------------------------------------------------
{
	int gain = 0;
	if(lowestExpoNoGain > 0)
	{
		float factor = -(gainMax - 1.0f) / (float)(lowestExpoNoGain - lowestExpo);
		gain = factor*(exposure - lowestExpo) + gainMax;
		if(gain < 0) gain = 0;
		if(gain > calibParams[IDX_CALIB_MAX_GAIN].parameter) gain = calibParams[IDX_CALIB_MAX_GAIN].parameter;
	}
	return gain;
}

// Determine the smoothing level based on spot size and group count
// TODO: some other factor that determines smoothing needs = brightness distribution?
//-----------------------------------------------------------------------------
int Calibration::determineSmoothing(Image &frame)
//-----------------------------------------------------------------------------
{
	float smoothing = 0;
	unsigned int maxCount = 0;
	for(Group& g : frame.groups)
	{
		if(g.pixelCount > maxCount) maxCount = g.pixelCount;
	}

	// Determine approximate smoothing based on spot size, tuned on experiments
	smoothing = sqrt(maxCount) / 6.0f;

	if(smoothing > calibParams[IDX_CALIB_MAX_SMOOTHING].parameter) smoothing = calibParams[IDX_CALIB_MAX_SMOOTHING].parameter;

	return smoothing;
}

// Used for self test: check that laser is actually on with good spot properties
//-----------------------------------------------------------------------------
bool Calibration::checkLaserOn(Group& calib)
//-----------------------------------------------------------------------------
{
	camera.requestFrame();
	if(camera.waitForFrame())
	{
		Image frame(camera, fileStream, pat_health_port);
		if(frame.histBrightest > calibParams[IDX_CALIB_MIN_BRIGHTNESS].parameter){
			if(frame.histBrightest > frame.histPeak){
				if(frame.histBrightest - frame.histPeak > calibParams[IDX_CALIB_GOOD_PEAKTOMAX_DISTANCE].parameter){
					int numGroups = frame.performPixelGrouping();
					if(numGroups > 0){
						Group& spot = frame.groups[0];
						if(spot.valueMax >= calibParams[IDX_CALIB_MIN_BRIGHTNESS].parameter){
							//copy spot properties to calib
							calib.x = frame.area.x + spot.x;
							calib.y = frame.area.y + spot.y;
							calib.valueMax = spot.valueMax;
							calib.valueSum = spot.valueSum;
							calib.pixelCount = spot.pixelCount;
							return true;
						} else{
							log(pat_health_port, fileStream, "In calibration.cpp Calibration::checkLaserOn - Check failed: (spot.valueMax = ", spot.valueMax, ") < (CALIB_MIN_BRIGHTNESS = ", calibParams[IDX_CALIB_MIN_BRIGHTNESS].parameter, ")");
						}
					} else{
						log(pat_health_port, fileStream, "In calibration.cpp Calibration::checkLaserOn - Check failed: (numGroups = ", numGroups, ") = 0");
					}
				} else{
					log(pat_health_port, fileStream, "In calibration.cpp Calibration::checkLaserOn - Check failed: (frame.histBrightest = ", frame.histBrightest, ") - ",
						"(frame.histPeak = ", frame.histPeak, ") = ", frame.histBrightest - frame.histPeak, 
						" <= (TRACK_GOOD_PEAKTOMAX_DISTANCE = ", calibParams[IDX_CALIB_GOOD_PEAKTOMAX_DISTANCE].parameter, ")");
				}
			} else{
				log(pat_health_port, fileStream, "In calibration.cpp Calibration::checkLaserOn - Check failed: (frame.histBrightest = ", frame.histBrightest, ") <= (frame.histPeak = ", frame.histPeak, ")");
			}
		} else{
			log(pat_health_port, fileStream, "In calibration.cpp Calibration::checkLaserOn - Check failed: (frame.histBrightest = ", frame.histBrightest, ") <= (CALIB_MIN_BRIGHTNESS = ", calibParams[IDX_CALIB_MIN_BRIGHTNESS].parameter, ")");
		}
	} else{
		log(pat_health_port, fileStream, "In calibration.cpp Calibration::checkLaserOn - Check failed: camera.waitForFrame() failed.");
	}
	return false; 
}

// Used for self test: check that laser is off
//-----------------------------------------------------------------------------
bool Calibration::checkLaserOff()
//-----------------------------------------------------------------------------
{
	camera.requestFrame();
	if(camera.waitForFrame())
	{
		Image frame(camera, fileStream, pat_health_port);
		if(frame.histBrightest <= calibParams[IDX_CALIB_MIN_BRIGHTNESS].parameter){
			return true;
		} else{
			log(pat_health_port, fileStream, "In calibration.cpp Calibration::checkLaserOff - Check failed: (frame.histBrightest = ", frame.histBrightest, ") > (CALIB_MIN_BRIGHTNESS = ", calibParams[IDX_CALIB_MIN_BRIGHTNESS].parameter, ")");
		}
	} else{
		log(pat_health_port, fileStream, "In calibration.cpp Calibration::checkLaserOff - Check failed: camera.waitForFrame() failed.");
	}
	return false; 
}

// Used for self test: check that FSM is working (spot is moving when commanded to and has decent properties)
//-----------------------------------------------------------------------------
bool Calibration::testFSM(Group& calib)
//-----------------------------------------------------------------------------
{
	int prev_x, prev_y, delta_x, delta_y;
	fsm.setNormalizedAngles(0,0);
	this_thread::sleep_for(chrono::milliseconds(CALIB_FSM_RISE_TIME));
	if(checkLaserOn(calib)){
		prev_x = calib.x;
		logImage(string("CMD_SELF_TEST_FSM_Center"), camera, fileStream, pat_health_port);

		// (0,0) Passed. Set to (1,0):
		fsm.setNormalizedAngles(1,0);
		this_thread::sleep_for(chrono::milliseconds(CALIB_FSM_RISE_TIME));
		if(checkLaserOn(calib)){
			delta_x = calib.x - prev_x;
			if(abs(delta_x) > calibParams[IDX_CALIB_FSM_DISPLACEMENT_TOL].parameter){
				prev_y = calib.y; 
				logImage(string("CMD_SELF_TEST_FSM_X"), camera, fileStream, pat_health_port);

				// (1,0) Passed. Set to (1,1):
				fsm.setNormalizedAngles(1,1);
				this_thread::sleep_for(chrono::milliseconds(CALIB_FSM_RISE_TIME));
				if(checkLaserOn(calib)){
					delta_y = calib.y - prev_y; 
					if(abs(delta_y) > calibParams[IDX_CALIB_FSM_DISPLACEMENT_TOL].parameter){
						prev_x = calib.x; 
						logImage(string("CMD_SELF_TEST_FSM_XY"), camera, fileStream, pat_health_port);

						// (1,1) Passed. Set to (0,1):
						fsm.setNormalizedAngles(0,1);
						this_thread::sleep_for(chrono::milliseconds(CALIB_FSM_RISE_TIME));
						if(checkLaserOn(calib)){
							delta_x = calib.x - prev_x; 
							if(abs(delta_x) > calibParams[IDX_CALIB_FSM_DISPLACEMENT_TOL].parameter){
								prev_y = calib.y; 
								logImage(string("CMD_SELF_TEST_FSM_Y"), camera, fileStream, pat_health_port);

								// (0,1) Passed. Set to (0,0) and return:
								fsm.setNormalizedAngles(0,0);
								this_thread::sleep_for(chrono::milliseconds(CALIB_FSM_RISE_TIME));
								if(checkLaserOn(calib)){
									delta_y = calib.y - prev_y;
									if(abs(delta_y) > calibParams[IDX_CALIB_FSM_DISPLACEMENT_TOL].parameter){
										return true;
									} else{
										log(pat_health_port, fileStream, "In calibration.cpp Calibration::testFSM at ending (0,0) - FSM Y displacement check failed: (|delta_y| =", abs(delta_y), ") <= (CALIB_FSM_DISPLACEMENT_TOL = ", calibParams[IDX_CALIB_FSM_DISPLACEMENT_TOL].parameter, ")");
									}
								} else{
									log(pat_health_port, fileStream, "In calibration.cpp Calibration::testFSM at ending (0,0) - checkLaserOn failed");
								}
							} else{
								log(pat_health_port, fileStream, "In calibration.cpp Calibration::testFSM at (0,1) - FSM X displacement check failed: (|delta_x| =", abs(delta_x), ") <= (CALIB_FSM_DISPLACEMENT_TOL = ", calibParams[IDX_CALIB_FSM_DISPLACEMENT_TOL].parameter, ")");
							}
						} else{
							log(pat_health_port, fileStream, "In calibration.cpp Calibration::testFSM at (0,1) - checkLaserOn failed");
						}
					} else{
						log(pat_health_port, fileStream, "In calibration.cpp Calibration::testFSM at (1,1) - FSM Y displacement check failed: (|delta_y| =", abs(delta_y), ") <= (CALIB_FSM_DISPLACEMENT_TOL = ", calibParams[IDX_CALIB_FSM_DISPLACEMENT_TOL].parameter, ")");
					}
				} else{
					log(pat_health_port, fileStream, "In calibration.cpp Calibration::testFSM at (1,1) - checkLaserOn failed");
				}
			} else{
				log(pat_health_port, fileStream, "In calibration.cpp Calibration::testFSM at (1,0) - FSM X displacement check failed: (|delta_x| =", abs(delta_x), ") <= (CALIB_FSM_DISPLACEMENT_TOL = ", calibParams[IDX_CALIB_FSM_DISPLACEMENT_TOL].parameter, ")");
			}
		} else{
			log(pat_health_port, fileStream, "In calibration.cpp Calibration::testFSM at (1,0) - checkLaserOn failed");
		}
	} else{
		log(pat_health_port, fileStream, "In calibration.cpp Calibration::testFSM at starting (0,0) - checkLaserOn failed");
	}
	return false;					
}