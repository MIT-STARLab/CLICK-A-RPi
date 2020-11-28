// Author: Peter Grenfell
#ifndef __PACKETDEF
#define __PACKETDEF

#include <zmq.hpp>
#include <iostream>
#define BUFFER_SIZE 256 //Needs to be long enough to fit all messages (I think longest one is 132)
#define WRITE 1 //(CLICK-A CPU Software Architecture on Google Drive)
#define READ 0 //(CLICK-A CPU Software Architecture on Google Drive)
#define CMD_START_PAT 0x00
#define CMD_END_PAT 0x01
#define CMD_PAYLOAD_SIZE 256
#define CMD_HEADER_SIZE 5

// Packet Definitions
struct fpga_request_packet_struct{
	uint32_t return_address;
	uint8_t request_number;
	bool read_write_flag;
	uint16_t start_address;
	uint32_t data_size;
	uint32_t data_to_write;
};

struct pat_health_packet_struct{
	uint32_t return_address;
	uint32_t data_size;
	char data_to_write[BUFFER_SIZE];
};

struct pat_control_packet_struct{
	char header[CMD_HEADER_SIZE];
	uint16_t command;
	uint16_t data_size;
	char data_to_read[CMD_PAYLOAD_SIZE];
};

void receive_packet(zmq::socket_t& sub_port, char* packet);

// Packet Sending for PUB Processes:
void send_packet_fpga_map_request(zmq::socket_t& pub_port, uint16_t channel, uint32_t data, bool read_write, uint8_t request_num);

void send_packet_pat_health(zmq::socket_t& pat_health_port, char* data);

// TODO: create_packet_tx definition (don't need to send bus commands for basic operation)

// Packet Receiving & Parsing for SUB Processes:
uint16_t receive_packet_pat_control(zmq::socket_t& pat_control_port, char* data_to_read = NULL);

// TODO: fpga_map_answer packet parsing helper function

// TODO: parse_packet_pat_control (commands from command handler)


#endif
