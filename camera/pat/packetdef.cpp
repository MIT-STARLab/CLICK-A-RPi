// Author: Peter Grenfell

#include "packetdef.h"

// Generic Code to Receive IPC Packet on SUB Port
void receive_packet(zmq::socket_t& sub_port, char* packet) //TODO: customize this for each type of packet (better to encapsulate packets for memory management)
{
	zmq::message_t message;
	sub_port.recv(message, zmq::recv_flags::none);
	memcpy(packet, message.data(), message.size());
}  

// Packet Sending for PUB Processes:

void send_packet_fpga_map_request(zmq::socket_t& fpga_map_request_port, uint16_t channel, uint32_t data, bool read_write, uint8_t request_num)
{
	fpga_request_packet_struct packet_struct = fpga_request_packet_struct();
	packet_struct.return_address = 3968; //Static PID: can replace with in-code console command to ps if dynamic PID is desired.
	packet_struct.request_number = request_num;
	packet_struct.read_write_flag = read_write;
	packet_struct.start_address = channel;
	packet_struct.data_size = sizeof(data); 
	packet_struct.data_to_write = data;
	
	char packet[sizeof(fpga_request_packet_struct)];
	memcpy(packet, &packet_struct, sizeof(packet));	   
	
	zmq::message_t message(sizeof(packet));
	memcpy(message.data(), packet, sizeof(packet));
	
	fpga_map_request_port.send(message);	
}

void send_packet_pat_health(zmq::socket_t& pat_health_port, char* data)
{
	pat_health_packet_struct packet_struct = pat_health_packet_struct();
	packet_struct.return_address = 3968; //Static PID: can replace with in-code console command to ps if dynamic PID is desired.
	packet_struct.data_size = sizeof(packet_struct.data_to_write);
	memcpy(packet_struct.data_to_write, data, strlen(data)+1);	
	
	char packet[sizeof(pat_health_packet_struct)];
	memcpy(packet, &packet_struct, sizeof(packet));	
	
	zmq::message_t message(sizeof(packet));
	memcpy(message.data(), packet, sizeof(packet));
	
	pat_health_port.send(message);	
}

// TODO: create_packet_tx definition (don't need to send bus commands for basic operation)

// Packet Receiving & Parsing for SUB Processes:

uint16_t receive_packet_pat_control(zmq::socket_t& pat_control_port, char* data_to_read)
{
	zmq::message_t message;
	pat_control_port.recv(message, zmq::recv_flags::none);
	
	char packet[sizeof(pat_control_packet_struct)];
	memcpy(packet, message.data(), message.size());
	
	pat_control_packet_struct packet_struct = pat_control_packet_struct(); //initialize
	memcpy(&packet_struct, packet, sizeof(packet));
	
	if(data_to_read != NULL){
		memcpy(data_to_read, packet_struct.data_to_read, CMD_PAYLOAD_SIZE);
	}
	
	//printf("\nPAT Control Packet Received (Internal): \nHeader: %s \nCommand: %d \nSize: %d \nData: %s \n", packet_struct.header, packet_struct.command, packet_struct.data_size, packet_struct.data_to_read);
	return packet_struct.command;
}

// TODO: fpga_map_answer packet parsing

// TODO: parse_packet_rx_pat (shouldn't need to receive bus commands for basic operation...)
