// Author: Peter Grenfell
#ifndef __PACKETDEF
#define __PACKETDEF

#include <zmq.hpp>

// Generic Code to Receive IPC Packet on SUB Port
char* receive_packet(zmq::socket_t& sub_port);

// Generic Code to Send IPC Packet on PUB Port  
void send_packet(zmq::socket_t& pub_port, char* packet);

char* create_packet_fpga_map_request_write(uint8_t channel, uint8_t data, uint8_t request_number);

// If needed, can put in a fpga_map_answer packet parsing helper function

// TODO: create_packet_tx definition (don't need to send bus commands for basic operation)

// TODO: parse_packet_rx_pat (shouldn't need to receive bus commands for basic operation...)

// TODO: create_packet_pat_health (logs to housekeeping)

// TODO: parse_packet_pat_control (commands from command handler)


#endif
