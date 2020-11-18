#include "fpga.h"

//All of this code needs to be replaced with zeromq inter-process communication to the C&DH, which talks to the FPGA.

//FPGA Constructor
FPGA::FPGA(std::ofstream &fileStreamIn) :
fileStream(fileStreamIn)
//-----------------------------------------------------------------------------
{
    //initialize FPGA
}
/*
handle(NULL), error(NULL), vp(NULL) //optional initialize vp to VID:PID, TBD
{
  status = flInitialise(0, &error); //initialise library
  check();

  status = flOpen(vp, &handle, &error); //Open connection of FPGALink device at specified VID and PID
  check();

  status = flIsDeviceAvailable(vp, &flag, &error); //check if FPGA is connected
  check();

  bool isCommCapable = flIsCommCapable(handle, 0x01); //check if comm capable
  if(!isCommCapable){
    log(std::err, fileStream, "FPGA Not Comm Capable.");
  }
  else{
    log(std::cout, fileStream, "Success, FPGA Comm Capable.")
  }

}
*/

//FPGA Destructor
FPGA::~FPGA(){
  //flClose(handle);
}

//error handling
/*
void FPGA::check(){
 if ( status != FL_SUCCESS ) {
   returnCode = x;
   log(std::cout, "FPGA Status Error:");
   log(std::cerr, error);
   flFreeError(error);
   goto cleanup;
 }
 */

//Turn on Calibration Laser, will be used in main
void FPGA::laserOn(){
  byte = CALIB_ON;
  //status = flWriteChannel(handle, CALIB_CH, sizeof(byte), &byte, &error);
  //check();
}

//Turn Off Calibration Laser, will be used in main
void FPGA::laserOff(){
  byte = CALIB_OFF;
  //status = flWriteChannel(handle, CALIB_CH, sizeof(byte), &byte, &error);
  //check();
}

//write to FSM, will be used by FSM class
void FPGA::fsmWrite(uint8_t channel, uint8_t &data){
  //status = flWriteChannel(handle, channel, sizeof(data), &error);
  //check();
}
