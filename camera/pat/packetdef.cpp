// Author: Peter Grenfell

#include "packetdef.h"

struct fpga_request_packet_struct{
	uint8_t fpga_return_address;
	uint8_t request_num;
	uint8_t write_flag;
	uint8_t start_address;
	uint8_t data_size;
	uint8_t data_to_write;
};

// Generic Code to Receive IPC Packet on SUB Port
char* receive_packet(zmq::socket_t& sub_port)
{
	zmq::message_t message;
    sub_port.recv(message, zmq::recv_flags::none);
    std::string message_str;
    message_str.assign(static_cast<char *>(message.data()), message.size());
    int n = message_str.length();
    char packet[n + 1];
    strcpy(packet, message_str.c_str());
    return packet;
}  

// Generic Code to Send IPC Packet on PUB Port  
void send_packet(zmq::socket_t& pub_port, char* packet)
{
	zmq::message_t message(strlen(packet));
	memcpy(message.data(), packet, strlen(packet));
	pub_port.send(message);
}
	
char* create_packet_fpga_map_request_write(uint8_t channel, uint8_t data, uint8_t request_number)
{
	fpga_request_packet_struct packet_struct = fpga_request_packet_struct();
	packet_struct.fpga_return_address = 0x4B;
	packet_struct.request_num = request_number;
	packet_struct.write_flag = 0x01;
	packet_struct.start_address = channel;
	packet_struct.data_size = sizeof(data);
	packet_struct.data_to_write = data;
	
	char packet[sizeof(fpga_request_packet_struct)];
	memcpy(packet, &packet_struct, sizeof(fpga_request_packet_struct));
	return packet;
		
	//char* packet = reinterpret_cast<char*>(packet_raw);
	
	/*
	//convert numerical packet data to strings
	std::string fpga_return_address = std::to_string(0x4B);
	std::string request_number_str = std::to_string(request_number);
	std::string write_flag = std::to_string(0x01); //(read is 0x00)
	std::string start_address = std::to_string(channel); 
	std::string data_to_write = std::to_string(data); 
	std::string data_size = std::to_string(data_to_write.length());	
	//concatenate packet sub-strings
	std::string packet_str = fpga_return_address + request_number_str + write_flag + start_address + data_size + data_to_write;
	
	//return packet character array
	int n = packet_str.length();
    char packet[n + 1];
    strcpy(packet, packet_str.c_str());  
    */
      
}

// If needed, can put in a fpga_map_answer packet parsing helper here:
// packet format:
// return address/flag
// request number
// read/write flag and error flag
// start address
// size
// data read

// TODO: create_packet_tx definition (don't need to send bus commands for basic operation)

// TODO: parse_packet_rx_pat (shouldn't need to receive bus commands for basic operation...)

// TODO: create_packet_pat_health (logs to housekeeping)

// TODO: parse_packet_pat_control (commands from command handler)
