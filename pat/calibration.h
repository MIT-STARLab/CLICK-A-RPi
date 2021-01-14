// CLICK A FSM Calibration & Calibration laser algorithms
// Authors: Ondrej Cierny, Peter Grenfell
#ifndef __CALIBRATION
#define __CALIBRATION
#include <vector>
#include "processing.h"
#include "fsm.h"
#include "log.h"

#define CALIB_FSM_MAX 1.0f/100		// Max FSM range to use during calibration / 100
#define CALIB_FSM_RISE_TIME 45		// FSM rise time in ms
#define CALIB_FSM_MAX_DELTA 0.003f	// Max difference in FSM output per step -> equivalent of moving ~3px on detector

#define CALIB_BIG_WINDOW 600		// Initial camera window for FSM 0,0 acquisition
#define CALIB_SMALL_WINDOW 300		// Camera window for calibration pattern tracking

#define CALIB_MIN_BRIGHTNESS 100	// Minimum brightness of calib laser spot to work with
#define CALIB_MAX_EXPOSURE 500 	// Max exposure in us to check for range tuning
#define CALIB_MAX_GAIN 20			// Max detector gain to allow
#define CALIB_EXP_DIVIDER 15		// Exposure tuning division factor, the higher the finer tuning, but slower; for range search
#define CALIB_MAX_SMOOTHING 3		// Maximum blurring of frames allowed
#define CALIB_GOOD_PEAKTOMAX_DISTANCE 100		// Minimum distance between histogram peak's brightness and maximum brightness
												// I.e. difference between most pixels (background) and active (brightest) pixels
#define CALIB_HAPPY_BRIGHTNESS 300				// Good brightness for auto exposure tuning

// A pair of source and destination points (Detector -> FSM)
class Pair
{
public:
	double x0, y0, x1, y1;
	Pair(double x0, double y0, double x1, double y1) : x0(x0), y0(y0), x1(x1), y1(y1) {}
};

// Base FSM calibration class
class Calibration
{
	Camera& camera;
	FSM& fsm;
	std::ofstream &fileStream;
	zmq::socket_t &pat_health_port;
	// Calculation functions
	void calculateSensitivityMatrix(std::vector<Pair>& data);
	void calculateAffineParameters(std::vector<Pair>& data);
	bool findExposureRange(Group& calib, std::string filePath = std::string("/root/log/pat/"));
public:
	// Affine transform parameters
	double a00, a01, a10, a11, t0, t1;
	// Sensitivity matrix
	double s00, s01, s10, s11;
	int preferredExpo, lowestExpo, lowestExpoNoGain, gainMax, smoothing, centerOffsetX, centerOffsetY;
	Calibration(Camera& camera, FSM& fsm, std::ofstream &fileStreamIn, zmq::socket_t &pat_health_port_in) : camera(camera), fsm(fsm), fileStream(fileStreamIn), pat_health_port(pat_health_port_in){};
	int gainForExposure(int exposure);
	int determineSmoothing(Image& frame);
	double transformDx(double x, double y);
	double transformDy(double x, double y);
	double affineTransformX(double x, double y);
	double affineTransformY(double x, double y);
	bool run(Group& calib, std::string filePath = std::string("/root/log/pat/"));
};

#endif
