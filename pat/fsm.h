// Mirrorcle MEMS FSM control class for the Raspberry Pi
// Based on Hyosang Yoon's Arduino controller
// Authors: Ondrej Cierny, Peter Grenfell
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

#define FSM_VBIAS_DAC 22000
#define FSM_VMAX_DAC 12000
#define FSM_FILTER 200

class FSM
{
	uint8_t spiBuffer[3];
	uint16_t voltageBias, voltageMax;
	int16_t oldX, oldY;
	uint8_t fsm_request_number = 0;
	std::ofstream &fileStream;
	zmq::socket_t &pat_health_port;
	zmq::socket_t &fpga_map_request_port;
	zmq::socket_t &fpga_map_answer_port;
	std::vector<zmq::pollitem_t>& poll_fpga_answer;
	void sendCommand(uint8_t cmd, uint8_t addr, uint16_t value);
	void sendCommand(uint32_t cmd);
	void fsmWrite(uint16_t channel, uint8_t data);
public:
	FSM(std::ofstream &fileStreamIn, zmq::socket_t &pat_health_port_in, zmq::socket_t& fpga_map_request_port_in, zmq::socket_t& fpga_map_answer_port_in, std::vector<zmq::pollitem_t>& poll_fpga_answer_in, uint16_t vBias_dac = FSM_VBIAS_DAC, uint16_t vMax_dac = FSM_VMAX_DAC, float filter = FSM_FILTER);
	~FSM();
	void setNormalizedAngles(float x, float y);
	void forceTransfer();
};

#endif
