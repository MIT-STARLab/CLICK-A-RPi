// Author: Peter Grenfell
#ifndef __PACKETDEF
#define __PACKETDEF

#include <zmq.hpp>
#define BUFFER_SIZE 512

// Packet Definitions
typedef struct fpga_request_packet_struct{
	uint32_t return_address;
	uint8_t request_num;
	bool write_flag;
	uint16_t start_address;
	uint32_t data_size;
	uint32_t data_to_write;
};

typedef struct pat_health_packet_struct{
	uint32_t return_address;
	uint32_t data_size;
	char data_to_write[BUFFER_SIZE];
};

// Generic Code to Receive IPC Packet on SUB Port
void receive_packet(zmq::socket_t& sub_port, char* packet);

// Generic Code to Send IPC Packet on PUB Port  
void send_packet(zmq::socket_t& pub_port, char* packet);

// Packet Creation for PUB Processes
char* create_packet_fpga_map_request_write(uint8_t channel, uint8_t data, uint8_t request_number);

void create_packet_pat_health(char* packet, char* data);

// TODO: create_packet_tx definition (don't need to send bus commands for basic operation)

// Packet Parsing for SUB Processes
// TODO: fpga_map_answer packet parsing helper function

// TODO: parse_packet_rx_pat (shouldn't need to receive bus commands for basic operation...)

// TODO: parse_packet_pat_control (commands from command handler)


#endif
