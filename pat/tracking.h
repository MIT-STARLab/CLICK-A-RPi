// CLICK A Tracking algorithms
// Authors: Ondrej Cierny, Peter Grenfell
#ifndef __TRACKING
#define __TRACKING
#include <chrono>
#include "camera.h"
#include "processing.h"
#include "calibration.h"
#include "log.h"

#define PERIOD_STATUS_MSG_TRACK 0.5f //seconds, time to wait in-between status messages

#define TRACK_GUESS_EXPOSURE 100 //guess at beacon exposure
#define TRACK_MIN_EXPOSURE 10 //minimum exposure limit, pg-comment
#define TRACK_MAX_EXPOSURE 100000 //maximum exposure limit, pg
#define TRACK_ACQUISITION_EXP_INCREMENT 300 //exposure increment during acquisition, pg
#define TRACK_MAX_GAIN 0 //maximum gain limit

#define TRACK_ACQUISITION_BRIGHTNESS 200		// Minimum spot brightness to work with for acquisition
#define TRACK_ACQUISITION_WINDOW 500			// Tuning camera window size
#define TRACK_GOOD_PEAKTOMAX_DISTANCE 100		// Minimum distance between histogram peak's brightness and maximum brightness
												// I.e. difference between most pixels (background) and active (brightest) pixels

#define TRACK_HAPPY_BRIGHTNESS 700				// Good brightness for auto exposure tuning
#define TRACK_SATURATION_LIMIT 990				// Camera is hard-saturated at around 990 brightness

#define TRACK_TUNING_MAX_ATTEMPTS 10			// Failsafe - give up tuning after 10 incorrect attempts (tuning noise?)
#define TRACK_TUNING_TOLERANCE 150				// Distance from TRACK_HAPPY_BRIGHTNESS that we are still happy with
#define TRACK_TUNING_POSITION_TOLERANCE 2		// Tolerance in location of spot in-between two tuning frames (jitter)
#define TRACK_TUNING_BRIGHTNESS_TOLERANCE 20	// Tolerance in brightness of spot in-between two tuning frames (jitter)
#define TRACK_TUNING_EXP_DIVIDER 10				// Exposure tuning division factor, the higher the finer tuning, but slower

#define TRACK_EXP_CONTROL_TOLERANCE 150		// Tolerance similar to above, but used in exposureControl function, pg
#define TRACK_EXP_CONTROL_DIVIDER 10			// Exposure tuning factor similar to above, but using in exposureControl function, pg

#define TRACK_WINDOW_SIZE_TOLERANCE 10			// If tracking window differs by more than this we definitely want an update
#define TRACK_MAX_SPOT_DIFFERENCE 500			// If spot parameters changed by too much since last update, something's wrong
#define TRACK_MIN_SPOT_LIMIT 25					// Minimum distance from spot to edge of adaptive window, i.e. assure safe distances

#define TRACK_CONTROL_I 8						// Controller integral constant
#define TRACK_CONTROL_MAX_TS 0.05f				// Max Ts to allow, prevent controller going crazy when out-of-sync

#define TRACK_SAFE_DISTANCE_ALLOW 200
#define TRACK_SAFE_DISTANCE_PANIC 100

using namespace std::chrono;

//for loading modifiable constant parameters from external file
enum trackParamIndex { 
    IDX_TRACK_GUESS_EXPOSURE, 
    IDX_TRACK_MIN_EXPOSURE,
    IDX_TRACK_MAX_EXPOSURE,
    IDX_TRACK_ACQUISITION_EXP_INCREMENT,
    IDX_TRACK_ACQUISITION_BRIGHTNESS,
    IDX_TRACK_ACQUISITION_WINDOW,
    IDX_TRACK_GOOD_PEAKTOMAX_DISTANCE,
	IDX_TRACK_HAPPY_BRIGHTNESS,
	IDX_TRACK_TUNING_TOLERANCE,
	IDX_TRACK_TUNING_EXP_DIVIDER,
	IDX_TRACK_EXP_CONTROL_TOLERANCE,
	IDX_TRACK_EXP_CONTROL_DIVIDER,
	IDX_TRACK_WINDOW_SIZE_TOLERANCE,
	IDX_TRACK_MAX_SPOT_DIFFERENCE,
	IDX_TRACK_MIN_SPOT_LIMIT,
	IDX_TRACK_CONTROL_I,
	IDX_TRACK_CONTROL_MAX_TS_MS,
	IDX_TRACK_SAFE_DISTANCE_ALLOW,
	IDX_TRACK_SAFE_DISTANCE_PANIC,
    NUM_TRACK_PARAMS
};
struct trackParamStruct {
    string name;
    int parameter;
};

//Tracking Class
class Tracking
{
	Camera& camera;
	Calibration& calibration;
	time_point<steady_clock> lastUpdate;
	std::ofstream &fileStream;
	zmq::socket_t &pat_status_port;
	zmq::socket_t &pat_health_port;
	zmq::socket_t &pat_control_port;
	std::vector<zmq::pollitem_t>& poll_pat_control; 
	double track_control_max_ts = TRACK_CONTROL_MAX_TS;
	bool verifyFrame(Image& frame, bool printFailure = false);
	bool windowAndTune(Image& frame, Group& beacon, AOI& beaconWindow, int maxBcnExposure = TRACK_MAX_EXPOSURE);
	bool autoTuneExposure(Group& beacon, int maxExposure = TRACK_MAX_EXPOSURE);
public:
	//int beaconSmoothing = 0;
	trackParamStruct trackParams[NUM_TRACK_PARAMS];
	double actionX, actionY;
	bool received_end_pat_cmd = false;
	bool received_end_process_cmd = false;
	Tracking(Camera& c, Calibration& calib, std::ofstream &fileStreamIn, zmq::socket_t &pat_status_port_in, zmq::socket_t &pat_health_port_in, zmq::socket_t& pat_control_port_in, std::vector<zmq::pollitem_t>& poll_pat_control_in);
	bool getTrackParams();
	bool runAcquisition(Group& beacon, AOI& beaconWindow, int maxExposure = TRACK_MAX_EXPOSURE);
	int findSpotCandidate(Image& frame, Group& oldSpot, double *difference);
	void updateTrackingWindow(Image& frame, Group& spot, AOI& window);
	void control(FSM& fsm, double x, double y, double spX, double spY);
	void controlOpenLoop(FSM& fsm, double x, double y);
	bool distanceIsSafe(Group& beacon, Group& calib, bool openloop);
	int controlExposure(int valueMax, int exposure, int maxBcnExposure = TRACK_MAX_EXPOSURE); //pg, exposure control function, returns updated exposure
};

#endif
