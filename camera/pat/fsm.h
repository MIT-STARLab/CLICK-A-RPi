// Mirrorcle MEMS FSM control class for the Raspberry Pi
// Based on Hyosang Yoon's Arduino controller
// Author: Ondrej Cierny
#ifndef __FSM
#define __FSM
#include <stdint.h>
//#include "defines.h"
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

#define FSM_A_CH 0x20 //fsm fpga channel a: fsm configuration
#define FSM_B_CH 0x21 //fsm fpga channel b: voltage 1
#define FSM_C_CH 0x22 //fsm fpga channel c: voltage 2

class FSM
{
	//int gpioHandle, spiHandle; nonflight
	char spiBuffer[3];
	//uint8_t enablePin; nonflight
	uint16_t voltageBias, voltageMax;
	int16_t oldX, oldY;
	void sendCommand(uint8_t cmd, uint8_t addr, uint16_t value);
	void sendCommand(uint32_t cmd);
	//void spiTransfer(); nonflight
	std::ofstream &fileStream;
	FPGA &fpga;
public:
	FSM(float vBias, float vMax, float filter, std::ofstream &fileStreamIn, FPGA &fpgaIn);
	//nonflight FSM args: (int gpioHandle, uint8_t spi, uint8_t pwmPin, uint8_t ePin, float vBias, float vMax, float filter, std::ofstream &fileStreamIn, FPGA &fpgaIn);
	~FSM();
	//void enableAmp(); nonflight
	//void disableAmp(); nonflight
	void setNormalizedAngles(float x, float y);
	void forceTransfer();
};

#endif