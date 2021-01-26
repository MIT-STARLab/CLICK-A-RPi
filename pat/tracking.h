// CLICK A Tracking algorithms
// Authors: Ondrej Cierny, Peter Grenfell
#ifndef __TRACKING
#define __TRACKING
#include <chrono>
#include "camera.h"
#include "processing.h"
#include "calibration.h"
#include "log.h"

#define TRACK_GUESS_EXPOSURE 1000 //guess at beacon exposure
#define TRACK_MIN_EXPOSURE 100 //minimum exposure limit, pg-comment
#define TRACK_MAX_EXPOSURE 10000 //maximum exposure limit, pg
#define TRACK_ACQUISITION_EXP_INCREMENT 100 //exposure increment during acquisition, pg
#define TRACK_MAX_GAIN 10 //maximum gain limit

#define TRACK_ACQUISITION_BRIGHTNESS 100		// Minimum spot brightness to work with for acquisition
#define TRACK_ACQUISITION_WINDOW 200			// Initial camera window size after acquisition is declared
#define TRACK_GOOD_PEAKTOMAX_DISTANCE 100		// Minimum distance between histogram peak's brightness and maximum brightness
												// I.e. difference between most pixels (background) and active (brightest) pixels

#define TRACK_HAPPY_BRIGHTNESS 200				// Good brightness for auto exposure tuning
#define TRACK_SATURATION_LIMIT 990				// Camera is hard-saturated at around 990 brightness

#define TRACK_TUNING_MAX_ATTEMPTS 10			// Failsafe - give up tuning after 10 incorrect attempts (tuning noise?)
#define TRACK_TUNING_TOLERANCE 50				// Distance from TRACK_HAPPY_BRIGHTNESS that we are still happy with
#define TRACK_TUNING_POSITION_TOLERANCE 2		// Tolerance in location of spot in-between two tuning frames (jitter)
#define TRACK_TUNING_BRIGHTNESS_TOLERANCE 20	// Tolerance in brightness of spot in-between two tuning frames (jitter)
#define TRACK_TUNING_EXP_DIVIDER 12				// Exposure tuning division factor, the higher the finer tuning, but slower

#define TRACK_EXP_CONTROL_TOLERANCE 50		// Tolerance similar to above, but used in exposureControl function, pg
#define TRACK_EXP_CONTROL_DIVIDER 10			// Exposure tuning factor similar to above, but using in exposureControl function, pg

#define TRACK_WINDOW_SIZE_TOLERANCE 10			// If tracking window differs by more than this we definitely want an update
#define TRACK_MAX_SPOT_DIFFERENCE 100			// If spot parameters changed by too much since last update, something's wrong
#define TRACK_MIN_SPOT_LIMIT 25					// Minimum distance from spot to edge of adaptive window, i.e. assure safe distances

#define TRACK_CONTROL_I 8						// Controller integral constant
#define TRACK_CONTROL_MAX_TS 0.05f				// Max Ts to allow, prevent controller going crazy when out-of-sync

#define TRACK_SAFE_DISTANCE_ALLOW 200
#define TRACK_SAFE_DISTANCE_PANIC 100

using namespace std::chrono;

class Tracking
{
	Camera& camera;
	Calibration& calibration;
	time_point<steady_clock> lastUpdate;
	std::ofstream &fileStream;
	zmq::socket_t &pat_health_port;
	zmq::socket_t &pat_control_port;
	std::vector<zmq::pollitem_t>& poll_pat_control; 
	bool verifyFrame(Image& frame);
	bool windowAndTune(Image& frame, Group& beacon);
	bool autoTuneExposure(Group& beacon);
public:
	int beaconSmoothing;
	double actionX, actionY;
	bool received_end_pat_cmd = false;
	Tracking(Camera& c, Calibration& calib, std::ofstream &fileStreamIn, zmq::socket_t &pat_health_port_in, zmq::socket_t& pat_control_port_in, std::vector<zmq::pollitem_t>& poll_pat_control_in) : camera(c), calibration(calib), fileStream(fileStreamIn), pat_health_port(pat_health_port_in), pat_control_port(pat_control_port_in), poll_pat_control(poll_pat_control_in), beaconSmoothing(0), actionX(0), actionY(0) {};
	bool runAcquisition(Group& beacon);
	int findSpotCandidate(Image& frame, Group& oldSpot, double *difference);
	void updateTrackingWindow(Image& frame, Group& spot, AOI& window);
	void control(FSM& fsm, double x, double y, double spX, double spY);
	void controlOpenLoop(FSM& fsm, double x, double y);
	bool distanceIsSafe(Group& beacon, Group& calib, bool openloop);
	int controlExposure(Image& frame, int exposure); //pg, exposure control function, returns updated exposure
};

#endif
