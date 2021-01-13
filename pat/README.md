## README for Pointing, Acquisition, and Tracking (PAT) Software
Authors: Ondrej Cierny & Peter Grenfell

## Basic Overview
The PAT software performs fine pointing control for the CLICK A payload's transmission laser. The ground station beacon beam spot is received by the camera and used for pointing feedback. The payload also produces an internal calibration beam that is coaxial with the transmit beam and received on the camera. The optical design is such that when the calibration beam spot is mirrored across the origin of the camera coordinate frame from the beacon spot, the transmit laser is coaxial with the beacon laser, which means that a laser communications link can be set up. The software maintains this configuration via feedback control throughout the duration of the experiment. 

## Execution Logic and Telemetry

pat:
- *Executable run via shell command: ./pat

packetdef.h, packetdef.cpp: 
- packet structure definitions and helper functions for zmq inter-process communication

log.h, log.cpp: helper functions for logging text telemetry & sending health messages to housekeeping. 
- Uses basic libraries only.
- Used by all of the above classes for text telemetry as well as during main.

lodepng.h, lodepng.cpp: 
- PNG image format library used for saving image telemetry from the camera

EasyBMP.h, EasyBMP.cpp, EasyBMP_VariousBMPutilities.h, EasyBMP_DataStructures.h, EasyBMP_BMP.h: 
- BMP image format library used for saving image telemetry from the camera (http://easybmp.sourceforge.net/).

main.cpp:
- The primary execution first initializes all needed objects, variables, and telemetry data structures and functions. The main loop uses a case structure that has 7 phases.
1. START: turn calibration laser on. Move to CALIBRATION phase.
2. CALIBRATION: use Calibration object function "run" to find appropriate camera settings and centroid the calibration spot. Save image telemetry. If failure occurs, return to START. Otherwise move to ACQUISITION phase.
3. ACQUISITION: Turn calibration laser off. Use Tracking object function "runAcquisition" to centroid the beacon spot. Use open loop tracking to maintain lock. If running in open loop mode, move to OPEN_LOOP phase; else, move to CL_INIT phase.
4. STATIC_POINT: Graceful failure mode for when beacon is not detected. Can also be directly commanded. Commands the FSM to point the laser straight & rely on bus pointing. Will not leave this phase until exit.
4. OPEN_LOOP: Only executes if the program is running in open loop mode. Maintains beacon lock without using calibration laser feedback (less accurate than closed loop). Will not leave this phase until exit.
5. CL_INIT: Closed-Loop Initialization for tracking of both spots (calibration and beacon). Move to CL_BEACON phase.
6. CL_BEACON: process new frame of beacon spot. If spot is lost, there is a timeout, which when finished will move back to ACQUISITION. Otherwise, move to CL_CALIB.
7. CL_CALIB: process new frame of calibration spot. Move to CL_BEACON. 

## Partner Processes for Inter-Process Communication:

CLICK-A-RPi/fpga/driver/fpga-ipc.py:
- FPGA driver and command line output for running PAT on flat sat via ssh (execute this before executing ./pat)

CLICK-A-RPi/housekeeping/housekeeping.py:
- Flight housekeeping SW for receiving PAT health telemetry (assumed to be running prior to ./pat)

CLICK-A-RPi/commandhandler/commandhandler.py:
- Flight command handling SW for sending commands to the PAT process (assumed to be running prior to ./pat)

CLICK-A-RPi/bus/businterface.py:
- Flight bus interface SW for sending ADCS feedback telemetry from PAT to the bus. (assumed to be running prior to ./pat)
- PAT process only sends feedback when commanded to do so. 

testIPC_PatHealth_control.py:
- Command line interface for running PAT on flat sat via ssh (execute this before executing ./pat)

testIPC_tx_adcs.py:
- Command line output for testing ADCS feedback from PAT on flat sat via ssh (execute this before executing ./pat)

testIPC.cpp: 
- Depreciated cpp IPC test script (slated for deletion)

## Hardware Abstraction
The hardware components used include the camera, calibration laser, fast steering mirror (fsm) and associated electronics (fpga, daughter board for driving the fsm). These primary classes are all initialized as objects during the initial setup phase in main.cpp before being passed (by reference) to the more abstract algorithm classes.

camera.h, camera.cpp:
- Camera class for Matrix Vision camera communications and control. 
- AOI class is helper data structure class (area of interest of camera frame). 
- Uses mvIMPACT library (mvIMPACT_CPP/mvIMPACT_acquire.h) for direct hardware interface. 
- Used by Image and Tracking classes

fpga.h, fpga.cpp: 
- FPGA class for FPGA communications to control the FSM and calibration laser. 
- Uses FPGALink library (libfpgalink.h) for direct hardware interface. 
- Used by FSM class and main.cpp (calibration laser toggling) 

fsm.h, fsm.cpp:
- FSM class for FSM communications and control via serial commands to the DAC (AD5664: http://www.analog.com/media/en/technical-documentation/data-sheets/AD5624_5664.pdf). 
- Uses FPGA class.
- Used by Calibration and Tracking classes

## Image Processing and Control Algorithms
These classes are the heart of the control system and use the hardware abstraction classes to implement the fine pointing control capability.

processing.h, processing.cpp: 
- Image class for image processing (histogram analysis, beam centroiding (threshold, find pixels, find centroids), image blurring).  -----Group class is a helper class for data storage and sorting during centroiding. 
- Uses Camera class.
- Used by Calibration and Tracking classes and main.cpp

calibration.h, calibration.cpp:
- Calibration class for finding appropriate camera settings for the calibration laser and estimating the plant transform (camera centroid to FSM angle). 
- Pair class is helper class for data storage (pair of source and destination points (Detector -> FSM)). 
- Uses Camera and FSM classes.
- Used by Tracking class and main.cpp

tracking.h, tracking.cpp:
- Tracking class for closed loop control of the fsm via the feedback from the camera. Also has open-loop functionality if operation without calibration laser is necessary. 
- Uses Camera, AOI, Image, Group, FSM, and Calibration classes
- Used by main.cpp
