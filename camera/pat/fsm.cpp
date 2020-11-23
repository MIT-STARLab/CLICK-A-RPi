// Mirrorcle MEMS FSM control class for Raspberry Pi
// Author: Ondrej Cierny
#include "fsm.h"
//#include "log.h"
//#include <pigpiod_if2.h>, nonflight
#include <unistd.h>
#include <cstdlib>
#include <chrono>
#include <thread>

// Initialize MEMS FSM control board over SPI; filter = cutoff in Hz
//nonflight arguments: (int gpio, uint8_t spi, uint8_t pwmPin, uint8_t ePin, float vBias, float vMax, float filter, std::ofstream &fileStreamIn)
//-----------------------------------------------------------------------------
FSM::FSM(float vBias, float vMax, float filter, std::ofstream &fileStreamIn, zmq::socket_t& fpga_pub_port_in) :
fileStream(fileStreamIn), fpga_pub_port(fpga_pub_port_in)
//-----------------------------------------------------------------------------
{
	// Voltage setting (ref. PicoAmp datasheet)
	voltageBias = (vBias/160)*65535;
	voltageMax = (vMax/160)*32768;

	// Initialize DAC - AD5664
	oldX = 1; oldY = 1;
	sendCommand(DAC_FULL_RESET);
	sendCommand(DAC_ENABLE_INTERNAL_REFERENCE);
	sendCommand(DAC_ENABLE_ALL_DAC_CHANNELS);
	sendCommand(DAC_ENABLE_SOFTWARE_LDAC);
	setNormalizedAngles(0, 0);
	
	/*//not for flight
	uint8_t channel = 0;
	uint32_t flags = 0;

	enablePin = ePin;
	gpioHandle = gpio;
	// Select correct SPI channel
	switch(spi)
	{
		case SPI0_CE0:
			break;
		case SPI0_CE1:
			channel = 1;
			break;
		case SPI1_CE0:
			flags = 0x100;
			break;
		case SPI1_CE1:
			channel = 1;
			flags = 0x100;
			break;
		case SPI1_CE2:
			channel = 2;
			flags = 0x100;
			break;
		default:
			log(std::cerr, fileStream,"Invalid SPI channel!");
			exit(1);
			break;
	}
	//test only, not for flight
	set_mode(gpio, enablePin, PI_OUTPUT);
	gpio_write(gpio, enablePin, 0);


	// Set up SPI communication, 7.8 MHz = reliable [http://www.bootc.net/archives/2012/05/19/spi-on-the-raspberry-pi-again/]
	spiHandle = spi_open(gpio, channel, 1000000, flags);
	if(spiHandle < 0)
	{
		log(std::cerr, fileStream,"Failed to set up SPI link!");
		exit(-1);
	}
// not for flight
	
	// Set up filter clock, output needs to be 60x cutoff
	set_mode(gpio, pwmPin, PI_ALT0);
	hardware_PWM(gpio, pwmPin, 60*filter, 500000);
*/

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
		//log(std::cout, fileStream,"Updating FSM position to", x, ",", y);
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
  	spiBuffer[0] = cmd | addr; //a
  	spiBuffer[1] = (value >> 8) & 0xFF; //b
  	spiBuffer[2] = value & 0xFF; //c
	fsmWrite(FSM_A_CH, spiBuffer[0]); //write to channel a
	fsmWrite(FSM_B_CH, spiBuffer[1]); //write to channel b
	fsmWrite(FSM_C_CH, spiBuffer[2]); //write to channel c
}

//-----------------------------------------------------------------------------
void FSM::sendCommand(uint32_t cmd)
//-----------------------------------------------------------------------------
{
	spiBuffer[0] = (cmd>>16) & 0xFF; //a
	spiBuffer[1] = (cmd>>8) & 0xFF; //b
	spiBuffer[2] = cmd & 0xFF; //c
	fsmWrite(FSM_A_CH, spiBuffer[0]); //write to channel a
	fsmWrite(FSM_B_CH, spiBuffer[1]); //write to channel b
	fsmWrite(FSM_C_CH, spiBuffer[2]); //write to channel c
}

//write to FSM
void FSM::fsmWrite(uint8_t channel, uint8_t data){
	//char* packet_fsmWrite = create_packet_fpga_map_request_write(channel, data, 0); //TBR request number argument
	//send_packet(fpga_pub_port, packet_fsmWrite);
}

//test setup only, not needed for flight
/*
//-----------------------------------------------------------------------------
void FSM::spiTransfer() 
//-----------------------------------------------------------------------------
//test only, not for flight hardware configuration
{
	int ret = spi_write(gpioHandle, spiHandle, spiBuffer, 3);
	switch(ret)
	{
		case PI_BAD_HANDLE:
			log(std::cerr, fileStream,"Error during SPI transfer: PI_BAD_HANDLE!");
			break;
		case PI_BAD_SPI_COUNT:
			log(std::cerr, fileStream,"Error during SPI transfer: PI_BAD_SPI_COUNT!");
			break;
		case PI_SPI_XFER_FAILED:
			log(std::cerr, fileStream,"Error during SPI transfer: PI_SPI_XFER_FAILED!");
			break;
		default:
			break;
	}
}
*/
/*
//-----------------------------------------------------------------------------
void FSM::enableAmp()
//-----------------------------------------------------------------------------
{
	gpio_write(gpioHandle, enablePin, 1);
}

//-----------------------------------------------------------------------------
void FSM::disableAmp()
//-----------------------------------------------------------------------------
{
	setNormalizedAngles(0, 0);
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	gpio_write(gpioHandle, enablePin, 0);
}
*/
