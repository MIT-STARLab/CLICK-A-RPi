// Author: Peter Grenfell

#include "packetdef.h"

// Packet Sending for PUB Processes:

void send_packet_fpga_map_request(zmq::socket_t& fpga_map_request_port, uint16_t channel, uint8_t data, bool read_write, uint8_t request_num)
{
	fpga_request_packet_struct packet_struct = fpga_request_packet_struct();
	packet_struct.return_address = 3968; //Static PID: can replace with in-code console command to ps if dynamic PID is desired.
	packet_struct.request_number = request_num;
	packet_struct.read_write_flag = read_write;
	packet_struct.start_address = channel;	
	packet_struct.data_to_write[0] = 0x00; //pre-padding
	packet_struct.data_to_write[1] = 0x00; //pre-padding
	packet_struct.data_to_write[2] = 0x00; //pre-padding
	packet_struct.data_to_write[3] = data;
	packet_struct.data_size = sizeof(packet_struct.data_to_write); 
	
	//~ std::cout << "packetdef - packet size: " << sizeof(fpga_request_packet_struct) << std::endl;
	//~ std::cout << "packetdef - return_address: " << packet_struct.return_address << std::endl;
	//~ std::cout << "packetdef - request_number: " << unsigned(packet_struct.request_number) << std::endl;
	//~ std::cout << "packetdef - read_write_flag: " << packet_struct.read_write_flag << std::endl;
	//~ std::cout << "packetdef - start_address: " << packet_struct.start_address << std::endl;
	//~ std::cout << "packetdef - data_size: " << packet_struct.data_size << std::endl;
	//~ std::cout << "packetdef - data_to_write: " << unsigned(packet_struct.data_to_write[3]) << std::endl;
	
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

void send_packet_tx_adcs(zmq::socket_t& tx_packets_port, float body_frame_x_angular_error_radians, float body_frame_y_angular_error_radians)
{
	pat_tx_adcs_packet_struct packet_struct = pat_tx_adcs_packet_struct();
	packet_struct.apid = TX_ADCS_APID;
	packet_struct.data_size = sizeof(packet_struct.data_to_write);
	
	uint32_t buffer_x_error_little_endian;
	memcpy(&buffer_x_error_little_endian, &body_frame_x_angular_error_radians, sizeof(buffer_x_error_little_endian));
	packet_struct.data_to_write[0] = htonl(buffer_x_error_little_endian); //convert to big endian
	
	uint32_t buffer_y_error_little_endian;
	memcpy(&buffer_y_error_little_endian, &body_frame_y_angular_error_radians, sizeof(buffer_y_error_little_endian));
	packet_struct.data_to_write[1] = htonl(buffer_y_error_little_endian); //convert to big endian
	
	float body_frame_z_angular_error_radians = 0; //set to zero - see bus pointing feedback user's guide (one drive)
	uint32_t buffer_z_error_little_endian;
	memcpy(&buffer_z_error_little_endian, &body_frame_z_angular_error_radians, sizeof(buffer_z_error_little_endian));
	packet_struct.data_to_write[2] = htonl(buffer_z_error_little_endian); //convert to big endian

	char packet[sizeof(pat_tx_adcs_packet_struct)];
	memcpy(packet, &packet_struct, sizeof(packet));	
	
	zmq::message_t message(sizeof(packet));
	memcpy(message.data(), packet, sizeof(packet));
	
	tx_packets_port.send(message);	
}

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

void receive_packet_fpga_map_answer(zmq::socket_t& fpga_map_answer_port)
{
	zmq::message_t message;
	fpga_map_answer_port.recv(message, zmq::recv_flags::none);
	
	char packet[sizeof(fpga_answer_packet_struct)];
	memcpy(packet, message.data(), message.size());

	std::cout << "packet - " << packet << std::endl;
	
	fpga_answer_packet_struct packet_struct = fpga_answer_packet_struct(); //initialize
	memcpy(&packet_struct, packet, sizeof(packet));
	
	//printf("\nFPGA Answer Packet Received (Internal): \nHeader: %s \nCommand: %d \nSize: %d \nData: %s \n", packet_struct.header, packet_struct.command, packet_struct.data_size, packet_struct.data_to_read);
	
	std::cout << "packetdef - packet size: " << sizeof(fpga_answer_packet_struct) << std::endl;
	std::cout << "packetdef - return_address: " << packet_struct.header << std::endl;
	std::cout << "packetdef - return_address: " << packet_struct.return_address << std::endl;
	std::cout << "packetdef - request_number: " << unsigned(packet_struct.request_number) << std::endl;
	std::cout << "packetdef - read_write_flag: " << packet_struct.read_write_flag << std::endl;
	std::cout << "packetdef - start_address: " << packet_struct.start_address << std::endl;
	std::cout << "packetdef - data_size: " << packet_struct.data_size << std::endl;
	std::cout << "packetdef - data_to_write: " << unsigned(packet_struct.data_to_read[3]) << std::endl;
	
	//return packet_struct.data_to_read[3];
}

// TODO: parse_packet_rx_pat (shouldn't need to receive bus commands for basic operation...)
