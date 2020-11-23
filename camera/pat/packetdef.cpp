// Author: Peter Grenfell

#include "packetdef.h"

// Generic Code to Receive IPC Packet on SUB Port
void receive_packet(zmq::socket_t& sub_port, char* packet, int packet_buffer_length)
{
	zmq::message_t message;
    sub_port.recv(message, zmq::recv_flags::none);
    char buffer[message.size()];
	memcpy(buffer, message.data(), message.size());
	memcpy(packet, buffer, sizeof(buffer));
	
	pat_health_packet_struct packet_pat_health_struct = pat_health_packet_struct();
	memcpy(&packet_pat_health_struct, message.data(), message.size());
	printf("Internal - Return Address: %d; Size: %d; Data: %s \n", packet_pat_health_struct.return_address, packet_pat_health_struct.data_size, packet_pat_health_struct.data_to_write);
		
    /*
    std::string message_str;
    message_str.assign(static_cast<char *>(message.data()), message.size());
    return message_str;
    */
    
    /*
    int n = message_str.length();
    char packet[n + 1];
    strcpy(packet, message_str.c_str());
    return packet;
    */
}  

// Generic Code to Send IPC Packet on PUB Port  
void send_packet(zmq::socket_t& pub_port, char* packet, int packet_buffer_length)
{
	char buffer[packet_buffer_length];
	memcpy(buffer, packet, sizeof(buffer));
	
	zmq::message_t message(sizeof(buffer));
	memcpy(message.data(), buffer, sizeof(buffer));
	
	pat_health_packet_struct packet_pat_health_struct = pat_health_packet_struct();
	memcpy(&packet_pat_health_struct, message.data(), message.size());
	printf("Internal Sender - Return Address: %d; Size: %d; Data: %s \n", packet_pat_health_struct.return_address, packet_pat_health_struct.data_size, packet_pat_health_struct.data_to_write);

	pub_port.send(message);
}

// Packet Creation for PUB Processes
char* create_packet_fpga_map_request_write(uint8_t channel, uint8_t data, uint8_t request_number)
{
	fpga_request_packet_struct packet_struct = fpga_request_packet_struct();
	packet_struct.return_address = 3968; //Static PID: can replace with in-code console command to ps if dynamic PID is desired.
	packet_struct.request_num = request_number;
	packet_struct.write_flag = 1;
	packet_struct.start_address = channel;
	packet_struct.data_size = sizeof(data);
	packet_struct.data_to_write = data;
	
	char packet[sizeof(fpga_request_packet_struct)];
	memcpy(packet, &packet_struct, sizeof(fpga_request_packet_struct));
	return packet;      
}

void create_packet_pat_health(char* packet, char* data, int packet_buffer_length)
{
	pat_health_packet_struct packet_struct = pat_health_packet_struct();
	packet_struct.return_address = 3968; //Static PID: can replace with in-code console command to ps if dynamic PID is desired.
	packet_struct.data_size = sizeof(data);
	memcpy(packet_struct.data_to_write, data, strlen(data)+1);
	
	char buffer[packet_buffer_length];
	memcpy(buffer, &packet_struct, sizeof(buffer));
	memcpy(packet, buffer, sizeof(buffer));
}

// TODO: create_packet_tx definition (don't need to send bus commands for basic operation)

// Packet Parsing for SUB Processes
// TODO: fpga_map_answer packet parsing helper here:
// packet format:
// return address/flag
// request number
// read/write flag and error flag
// start address
// size
// data read

// TODO: parse_packet_rx_pat (shouldn't need to receive bus commands for basic operation...)

// TODO: parse_packet_pat_control (commands from command handler)
