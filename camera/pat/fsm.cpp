// Mirrorcle MEMS FSM control class for Raspberry Pi
// Authors: Ondrej Cierny, Peter Grenfell
#include "fsm.h"
#include <unistd.h>
#include <cstdlib>
#include <chrono>
#include <thread>

// Initialize MEMS FSM control board over SPI; filter = cutoff in Hz
//-----------------------------------------------------------------------------
FSM::FSM(std::ofstream &fileStreamIn, zmq::socket_t &pat_health_port_in, zmq::socket_t& fpga_map_request_port_in, float vBias, float vMax, float filter) :
fileStream(fileStreamIn), pat_health_port(pat_health_port_in), fpga_map_request_port(fpga_map_request_port_in)
//-----------------------------------------------------------------------------
{
	// Voltage setting (ref. PicoAmp datasheet)
	voltageBias = (vBias/160)*65535;
	voltageMax = (vMax/160)*32768;

	// Initialize DAC - AD5664
	log(pat_health_port, fileStream, "Initializing FSM DAC...");
	oldX = 1; oldY = 1;
	sendCommand(DAC_FULL_RESET);
	sendCommand(DAC_ENABLE_INTERNAL_REFERENCE);
	sendCommand(DAC_ENABLE_ALL_DAC_CHANNELS);
	sendCommand(DAC_ENABLE_SOFTWARE_LDAC);
	setNormalizedAngles(0, 0);
}

//-----------------------------------------------------------------------------
FSM::~FSM()
//-----------------------------------------------------------------------------
{
	setNormalizedAngles(0, 0); //reset to zero before destruction
}

//-----------------------------------------------------------------------------
void FSM::setNormalizedAngles(float x, float y)
//-----------------------------------------------------------------------------
{
	// Clamp to limits
	x = std::max(-1.0f, std::min(x, 1.0f));
	y = std::max(-1.0f, std::min(y, 1.0f));

	int16_t newX = ((int)(voltageMax * x)) & 0xFFFF;
	int16_t newY = ((int)(voltageMax * y)) & 0xFFFF;

	// Write X+, X-, Y+, Y- & Update
	if(newX != oldX || newY != oldY)
	{
		//log(pat_health_port, fileStream,"Updating FSM position to", x, ",", y);
		sendCommand(DAC_CMD_WRITE_INPUT_REG, DAC_ADDR_XP, voltageBias + newX);
		sendCommand(DAC_CMD_WRITE_INPUT_REG, DAC_ADDR_XM, voltageBias - newX);
		sendCommand(DAC_CMD_WRITE_INPUT_REG, DAC_ADDR_YP, voltageBias + newY);
		sendCommand(DAC_CMD_WRITE_INPUT_UPDATE_ALL, DAC_ADDR_YM, voltageBias - newY);
		oldX = newX;
		oldY = newY;
	}
}

//-----------------------------------------------------------------------------
void FSM::forceTransfer()
//-----------------------------------------------------------------------------
{
	sendCommand(DAC_CMD_WRITE_INPUT_REG, DAC_ADDR_XP, voltageBias + oldX);
	sendCommand(DAC_CMD_WRITE_INPUT_REG, DAC_ADDR_XM, voltageBias - oldX);
	sendCommand(DAC_CMD_WRITE_INPUT_REG, DAC_ADDR_YP, voltageBias + oldY);
	sendCommand(DAC_CMD_WRITE_INPUT_UPDATE_ALL, DAC_ADDR_YM, voltageBias - oldY);
}

//-----------------------------------------------------------------------------
void FSM::sendCommand(uint8_t cmd, uint8_t addr, uint16_t value)
//-----------------------------------------------------------------------------
{
  	spiBuffer[0] = cmd | addr; //a, bitwise OR operation
  	spiBuffer[1] = (value >> 8) & 0xFF; //b, shift value to the right by 8 bits and mask to last 8 bits
  	spiBuffer[2] = value & 0xFF; //c, value to last 8 bits
	fsmWrite(FSM_A_CH, spiBuffer[0]); //write to channel a
	fsmWrite(FSM_B_CH, spiBuffer[1]); //write to channel b
	fsmWrite(FSM_C_CH, spiBuffer[2]); //write to channel c
}

//-----------------------------------------------------------------------------
void FSM::sendCommand(uint32_t cmd)
//-----------------------------------------------------------------------------
{
	spiBuffer[0] = (cmd>>16) & 0xFF; //a, shift cmd to the right by 16 bits and mask to last 8 bits
	spiBuffer[1] = (cmd>>8) & 0xFF; //b, shift cmd to the right by 8 bits and mask to last 8 bits
	spiBuffer[2] = cmd & 0xFF; //c, mask to last 8 bits
	fsmWrite(FSM_A_CH, spiBuffer[0]); //write to channel a
	fsmWrite(FSM_B_CH, spiBuffer[1]); //write to channel b
	fsmWrite(FSM_C_CH, spiBuffer[2]); //write to channel c
}

//-----------------------------------------------------------------------------
//write to FSM
void FSM::fsmWrite(uint16_t channel, uint8_t data)
//-----------------------------------------------------------------------------
{
	send_packet_fpga_map_request(fpga_map_request_port, channel, data, WRITE, 0); //TBR request_number
}
