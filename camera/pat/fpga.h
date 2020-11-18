#ifndef FPGA_H
#define FPGA_H

//#include <libfpgalink.h>
#include "log.h"

#define CALIB_CH 0x16 //calib laser fpga channel
#define CALIB_ON 0x55 //calib laser ON code
#define CALIB_OFF 0x0F //calib laser OFF code

/* //only needed in fsm class, moved there
#define FSM_A_CH 0x20 //fsm fpga channel a: fsm configuration
#define FSM_B_CH 0x21 //fsm fpga channel b: voltage 1
#define FSM_C_CH 0x22 //fsm fpga channel c: voltage 2
*/

class FPGA
{
  //for initialization list
  /*
  struct FLContext *handle;
  const char *error;
  const char *vp; //VID:PID
  FLStatus status;
  */
  uint8_t flag;
  uint8_t byte;
  std::ofstream &fileStream;

public:
  FPGA(std::ofstream &fileStreamIn);
  ~FPGA();
  void laserOn();
  void laserOff();
  void fsmWrite(uint8_t channel, uint8_t data);

};

#endif
