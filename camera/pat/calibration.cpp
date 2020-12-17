// NODE FSM Calibration & Calibration laser algorithms
// Authors: Ondrej Cierny, Peter Grenfell
#include "calibration.h"
#include "tracking.h"
#include <chrono>
#include <thread>
#include <cmath>

// Find range of exposure settings where calibration spot has good parameters
// Also find the gains (function) associated with this range
// Also save properties of spot for re-use in main routine
//-----------------------------------------------------------------------------
bool Calibration::findExposureRange(Group& calib)
//-----------------------------------------------------------------------------
{
	int gain = 0, exposure = CALIB_MAX_EXPOSURE/10, skip = camera.queuedCount;
	preferredExpo = CALIB_MAX_EXPOSURE + 1;
	lowestExpoNoGain = 0;

	// Configure camera for max exposure and big window around center
	camera.config->gain_dB.write(gain);
	camera.config->binningMode.write(cbmOff);
	camera.config->expose_us.write(exposure);
	camera.setCenteredWindow(CAMERA_WIDTH/2, CAMERA_HEIGHT/2, CALIB_BIG_WINDOW);
	
    //logImage(string("CALIBRATION_debug"), camera, fileStream, pat_health_port);
    
    camera.requestFrame();

	// Skip pre-queued old frames
	camera.ignoreNextFrames(skip);

	if(camera.waitForFrame())
	{
		Image init(camera, fileStream, pat_health_port, smoothing);
		if(init.histBrightest <= CALIB_MIN_BRIGHTNESS){
			log(pat_health_port, fileStream, "In calibration.cpp Calibration::findExposureRange - Error: ", 
			"(init.histBrightest = ", init.histBrightest, ") <= (CALIB_MIN_BRIGHTNESS = ", CALIB_MIN_BRIGHTNESS, ")");
			//save image debug telemetry
			std::string nameTag = std::string("DEBUG_findExposureRange");
			std::string imageFileName = timeStamp() + std::string("_") + nameTag + std::string("_exp_") + std::to_string(camera.config->expose_us.read()) + std::string(".png");
			log(pat_health_port, fileStream, "In calibration.cpp Calibration::findExposureRange - Saving image telemetry as: ", imageFileName);
			init.savePNG(imageFileName);
			return false;
		}

		if(init.histBrightest <= init.histPeak){
			log(pat_health_port, fileStream, "In calibration.cpp Calibration::findExposureRange - Error: ", 
			"(init.histBrightest = ", init.histBrightest, ") <= (init.histPeak = ", init.histPeak, ")");
			//save image debug telemetry
			std::string nameTag = std::string("DEBUG_findExposureRange");
			std::string imageFileName = timeStamp() + std::string("_") + nameTag + std::string("_exp_") + std::to_string(camera.config->expose_us.read()) + std::string(".png");
			log(pat_health_port, fileStream, "In calibration.cpp Calibration::findExposureRange - Saving image telemetry as: ", imageFileName);
			init.savePNG(imageFileName);
			return false;
		}

		if(init.histBrightest - init.histPeak <= TRACK_GOOD_PEAKTOMAX_DISTANCE){
			log(pat_health_port, fileStream, "In calibration.cpp Calibration::findExposureRange - Error: ", 
			"(init.histBrightest = ", init.histBrightest, ") - ",
			"(init.histPeak = ", init.histPeak, ") = ", init.histBrightest - init.histPeak, 
			" <= (TRACK_GOOD_PEAKTOMAX_DISTANCE = ", TRACK_GOOD_PEAKTOMAX_DISTANCE, ")");
			//save image debug telemetry
			std::string nameTag = std::string("DEBUG_findExposureRange");
			std::string imageFileName = timeStamp() + std::string("_") + nameTag + std::string("_exp_") + std::to_string(camera.config->expose_us.read()) + std::string(".png");
			log(pat_health_port, fileStream, "In calibration.cpp Calibration::findExposureRange - Saving image telemetry as: ", imageFileName);
			init.savePNG(imageFileName);
			return false;
		}

		/*
		init.histBrightest > CALIB_MIN_BRIGHTNESS &&
		init.histBrightest > init.histPeak &&
		init.histBrightest - init.histPeak > TRACK_GOOD_PEAKTOMAX_DISTANCE &&
		init.performPixelGrouping() > 0
		*/
		int num_groups = init.performPixelGrouping();
		if(num_groups > 0)
		{
			// Check initial image values
			Group& spot = init.groups[0];

			// Set small window on spot location and search for range
			camera.setCenteredWindow(init.area.x + spot.x, init.area.y + spot.y, CALIB_SMALL_WINDOW);
			log(pat_health_port, fileStream, "In calibration.cpp Calibration::findExposureRange - Looking for calib expo range, starting with max", CALIB_MAX_EXPOSURE, "us at [",
				init.area.x + spot.x, ",", init.area.y + spot.y, "] with smoothing", smoothing);

			for(; exposure >= 10 && exposure/CALIB_EXP_DIVIDER >= 1; exposure -= exposure/CALIB_EXP_DIVIDER)
			{
				// Alter exposure
				camera.config->expose_us.write(exposure);
				camera.requestFrame();

				if(camera.waitForFrame())
				{
					Image frame(camera, fileStream, pat_health_port, smoothing);
					if(frame.histBrightest > frame.histPeak &&
					   frame.histBrightest - frame.histPeak > TRACK_GOOD_PEAKTOMAX_DISTANCE &&
					   frame.performPixelGrouping() > 0)
					{
						spot = frame.groups[0];
						// Find preferred exposure time (good brightness with no gain)
						if(preferredExpo > CALIB_MAX_EXPOSURE && spot.valueMax < TRACK_HAPPY_BRIGHTNESS)
						{
							log(pat_health_port, fileStream, "In calibration.cpp Calibration::findExposureRange - Found preferred exposure:", exposure, "us,", gain, "dB");
							preferredExpo = exposure;
							if(smoothing == 0)
							{
								int test = determineSmoothing(frame);
								if(test != smoothing)
								{
									smoothing = test;
									return true;
								}
							}
						}
						// Find lowest acceptable exposure where no gain is needed
						if(gain == 0 && lowestExpoNoGain == 0 && spot.valueMax <= CALIB_MIN_BRIGHTNESS)
						{
							log(pat_health_port, fileStream, "In calibration.cpp Calibration::findExposureRange - Found lowest no-gain exposure:", exposure, "us");
							lowestExpoNoGain = exposure;
						}
						// Check for lowest acceptable exposure with gain
						if(spot.valueMax <= CALIB_MIN_BRIGHTNESS)
						{
							if(gain < CALIB_MAX_GAIN)
							{
								gain++;
								camera.config->gain_dB.write(gain);
							}
							else break;
						}
					}
					else
					{
						log(pat_health_port, fileStream, "In calibration.cpp Calibration::findExposureRange - Lost spot during search! Peak is at", frame.histPeak, "brightest is", frame.histBrightest);
					}
				}
				else return false;
			}
			log(pat_health_port, fileStream, "In calibration.cpp Calibration::findExposureRange - Finished range search, lowest exposure", exposure, "us needs", gain, "dB");
			lowestExpo = exposure;
			gainMax = gain;
			if(gain == 0 && preferredExpo > CALIB_MAX_EXPOSURE) preferredExpo = exposure;
			// Copy over group parameters
			calib.valueMax = spot.valueMax;
			calib.valueSum = spot.valueSum;
			calib.pixelCount = spot.pixelCount;
			return true;
		}
		else{
			log(pat_health_port, fileStream, "In calibration.cpp Calibration::findExposureRange - Error: (num_groups = ", num_groups,") <= 0");
			//save image debug telemetry
			std::string nameTag = std::string("DEBUG_findExposureRange");
			std::string imageFileName = timeStamp() + std::string("_") + nameTag + std::string("_exp_") + std::to_string(camera.config->expose_us.read()) + std::string(".png");
			log(pat_health_port, fileStream, "In processing.cpp logImage - Saving image telemetry as: ", imageFileName);
			init.savePNG(imageFileName);
		}
	}
	else{
		log(pat_health_port, fileStream, "In calibration.cpp Calibration::findExposureRange - camera.waitForFrame() Failed");
	}
	return false;
}

// Finds acceptable camera settings for calib laser
// Performs a spiral calibration pattern on detector with FSM
// Collects 100 point pairs and calculates Detector -> FSM transformation parameters
//-----------------------------------------------------------------------------
bool Calibration::run(Group& calib)
//-----------------------------------------------------------------------------
{
	std::vector<Pair> points;
	double omega = 2 * M_PI * 0.05f;

	// Reset FSM to center
	fsm.setNormalizedAngles(0, 0);
	this_thread::sleep_for(chrono::milliseconds(CALIB_FSM_RISE_TIME));

	// Find the exposure setting range
	smoothing = 0;
	bool success = findExposureRange(calib);
	//if(smoothing != 0) success = findExposureRange(calib);

	if(success)
	{
		// Use preferred exposure
		camera.config->gain_dB.write(0);
		camera.config->expose_us.write(preferredExpo);
		logImage(string("CALIBRATION_Start"), camera, fileStream, pat_health_port); 
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
					if(i % 5 == 0){
						log(pat_health_port, fileStream, "In calibration.cpp Calibration::run - Pair", i, "[", frame.area.x + spot.x, ",", frame.area.y + spot.y, "] for FSM [", x, ",", y, "]");
					}

					// Move window to new location
					camera.setCenteredWindow(frame.area.x + spot.x, frame.area.y + spot.y, CALIB_SMALL_WINDOW);
				}
				else
				{
					log(pat_health_port, fileStream, "In calibration.cpp Calibration::run - Lost spot during FSM calibration!");
				}
			}
		}
		
		logImage(string("CALIBRATION_End"), camera, fileStream, pat_health_port); 

		// Reset FSM & Camera
		camera.setFullWindow();
		fsm.setNormalizedAngles(0, 0);

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
		if(gain > CALIB_MAX_GAIN) gain = CALIB_MAX_GAIN;
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

	if(smoothing > CALIB_MAX_SMOOTHING) smoothing = CALIB_MAX_SMOOTHING;

	return smoothing;
}
