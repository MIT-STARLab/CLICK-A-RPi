// Author: Peter Grenfell

#include "packetdef.h"

// Generic Code to Receive IPC Packet on SUB Port
void receive_packet(zmq::socket_t& sub_port, char* packet)
{
	zmq::message_t message;
	sub_port.recv(message, zmq::recv_flags::none);
	memcpy(packet, message.data(), message.size());
}  

// Generic Code to Send IPC Packet on PUB Port  
void send_packet(zmq::socket_t& pub_port, char* packet)
{
	char buffer[BUFFER_SIZE];
	memcpy(buffer, packet, sizeof(buffer));
	
	zmq::message_t message(sizeof(buffer));
	memcpy(message.data(), buffer, sizeof(buffer));
		
	pub_port.send(message);
}

// Packet Creation for PUB Processes:
void create_packet_fpga_map_request(char* packet, uint16_t channel, uint32_t data, bool read_write, uint8_t request_num)
{
	fpga_request_packet_struct packet_struct = fpga_request_packet_struct();
	packet_struct.return_address = 3968; //Static PID: can replace with in-code console command to ps if dynamic PID is desired.
	packet_struct.request_number = request_num;
	packet_struct.read_write_flag = read_write;
	packet_struct.start_address = channel;
	packet_struct.data_size = sizeof(data);
	packet_struct.data_to_write = data;
	
	char buffer[BUFFER_SIZE];
	memcpy(buffer, &packet_struct, sizeof(buffer));	
	memcpy(packet, buffer, sizeof(buffer));    
}

void create_packet_pat_health(char* packet, char* data)
{
	pat_health_packet_struct packet_struct = pat_health_packet_struct();
	packet_struct.return_address = 3968; //Static PID: can replace with in-code console command to ps if dynamic PID is desired.
	packet_struct.data_size = sizeof(data);
	memcpy(packet_struct.data_to_write, data, strlen(data)+1);	
	
	char buffer[BUFFER_SIZE];
	memcpy(buffer, &packet_struct, sizeof(buffer));	
	memcpy(packet, buffer, sizeof(buffer));
}

// TODO: create_packet_tx definition (don't need to send bus commands for basic operation)

// Packet Parsing for SUB Processes
// TODO: fpga_map_answer packet parsing

// TODO: parse_packet_rx_pat (shouldn't need to receive bus commands for basic operation...)

// TODO: parse_packet_pat_control (commands from command handler)
