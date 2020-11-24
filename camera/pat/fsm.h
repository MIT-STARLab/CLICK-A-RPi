// Mirrorcle MEMS FSM control class for the Raspberry Pi
// Based on Hyosang Yoon's Arduino controller
// Author: Ondrej Cierny
#ifndef __FSM
#define __FSM
#include <stdint.h>
#include "log.h"

#define DAC_ADDR_XP						0x00
#define DAC_ADDR_XM						0x01
#define DAC_ADDR_YM						0x02
#define DAC_ADDR_YP						0x03
#define DAC_FULL_RESET					0x280001
#define DAC_ENABLE_INTERNAL_REFERENCE	0x380001
#define DAC_ENABLE_ALL_DAC_CHANNELS		0x20000F
#define DAC_ENABLE_SOFTWARE_LDAC		0x300000
#define DAC_CMD_WRITE_INPUT_REG			0x00
#define DAC_CMD_WRITE_INPUT_UPDATE_ALL	(0x02 << 3)

#define FSM_A_CH 0x08 //fsm fpga channel a: fsm configuration (Notated_memory_map on Google Drive)
#define FSM_B_CH 0x09 //fsm fpga channel b: voltage 1 (Notated_memory_map on Google Drive)
#define FSM_C_CH 0x0A //fsm fpga channel c: voltage 2 (Notated_memory_map on Google Drive)

#define FSM_VBIAS 80
#define FSM_VMAX 129
#define FSM_FILTER 200

class FSM
{
	uint8_t spiBuffer[3];
	uint16_t voltageBias, voltageMax;
	int16_t oldX, oldY;
	std::ofstream &fileStream;
	zmq::socket_t &fpga_map_request_port;
	void sendCommand(uint8_t cmd, uint8_t addr, uint16_t value);
	void sendCommand(uint32_t cmd);
	void fsmWrite(uint16_t channel, uint8_t data);
	
public:
	FSM(std::ofstream &fileStreamIn, zmq::socket_t& fpga_map_request_port_in, float vBias = FSM_VBIAS, float vMax = FSM_VMAX, float filter = FSM_FILTER);
	~FSM();
	void setNormalizedAngles(float x, float y);
	void forceTransfer();
};

#endif
