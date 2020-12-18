// Author: Peter Grenfell
#ifndef __PACKETDEF
#define __PACKETDEF

#include <zmq.hpp>
#include <iostream>
#include <arpa/inet.h>
#define TX_ADCS_APID 0x250 //(CLICK-A CPU Software Architecture on Google Drive)
#define BUFFER_SIZE 256 //Needs to be long enough to fit all messages (I think longest one is 132)
#define WRITE 1 //(CLICK-A CPU Software Architecture on Google Drive)
#define READ 0 //(CLICK-A CPU Software Architecture on Google Drive)
#define CMD_PAYLOAD_SIZE 256
#define CMD_HEADER_SIZE 5
#define CMD_START_PAT 0x01
#define CMD_END_PAT 0x02
#define CMD_START_PAT_OPEN_LOOP 0x03
#define CMD_START_PAT_STATIC_POINT 0x04
#define CMD_START_PAT_BUS_FEEDBACK 0x05
#define CMD_GET_IMAGE 0x06
#define CMD_CALIB_TEST 0x07
#define CMD_CALIB_LASER_TEST 0x08
#define CMD_FSM_TEST 0x09

// Packet Definitions
struct fpga_request_packet_struct{
	uint32_t return_address;
	uint8_t request_number;
	bool read_write_flag;
	uint16_t start_address;
	uint32_t data_size;
	uint8_t data_to_write[4]; //struct alignment forces this to be 4 bytes, so need to add pre-padding since only have 1 byte of significant data
};

struct pat_health_packet_struct{
	uint32_t return_address;
	uint32_t data_size;
	char data_to_write[BUFFER_SIZE];
};

struct pat_tx_adcs_packet_struct{
	uint16_t apid;
	uint16_t data_size;
	uint32_t data_to_write[3];
};

struct pat_control_packet_struct{
	char header[CMD_HEADER_SIZE];
	uint16_t command;
	uint16_t data_size;
	char data_to_read[CMD_PAYLOAD_SIZE];
};

struct fpga_answer_packet_struct{
	char header[CMD_HEADER_SIZE];
	uint32_t return_address;
	uint8_t request_number;
	bool read_write_flag;
	uint16_t start_address;
	uint32_t data_size;
	uint8_t data_to_read[4]; //struct alignment forces this to be 4 bytes, so need to add pre-padding since only have 1 byte of significant data
};

// Packet Sending for PUB Processes:
void send_packet_fpga_map_request(zmq::socket_t& fpga_map_request_port, uint16_t channel, uint8_t data, bool read_write, uint8_t request_num);

void send_packet_pat_health(zmq::socket_t& pat_health_port, char* data);

void send_packet_tx_adcs(zmq::socket_t& tx_packets_port, float body_frame_x_angular_error_radians, float body_frame_y_angular_error_radians);

// Packet Receiving & Parsing for SUB Processes:
uint16_t receive_packet_pat_control(zmq::socket_t& pat_control_port, char* data_to_read = NULL);

void receive_packet_fpga_map_answer(zmq::socket_t& fpga_map_answer_port);

// TODO: receive_packet_pat_rx (commands from bus)


#endif
