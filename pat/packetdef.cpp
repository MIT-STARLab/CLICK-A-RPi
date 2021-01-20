// Author: Peter Grenfell

#include "packetdef.h"

// Packet Sending for PUB Processes:

void send_packet_fpga_map_request(zmq::socket_t& fpga_map_request_port, uint16_t channel, uint8_t data, bool read_write, uint8_t request_num)
{
	if(read_write == WRITE){
		fpga_request_write_packet_struct packet_struct = fpga_request_write_packet_struct();
		packet_struct.return_address = (uint32_t) getpid(); //get process pid
		packet_struct.request_number = request_num;
		packet_struct.read_write_flag = read_write;
		packet_struct.start_address = channel;	
		packet_struct.data_to_write[0] = 0x00; //pre-padding
		packet_struct.data_to_write[1] = 0x00; //pre-padding
		packet_struct.data_to_write[2] = 0x00; //pre-padding
		packet_struct.data_to_write[3] = data;
		packet_struct.data_size = sizeof(packet_struct.data_to_write); 
		
		//~ std::cout << "packetdef - packet size: " << sizeof(fpga_request_write_packet_struct) << std::endl;
		//~ std::cout << "packetdef - return_address: " << packet_struct.return_address << std::endl;
		//~ std::cout << "packetdef - request_number: " << unsigned(packet_struct.request_number) << std::endl;
		//~ std::cout << "packetdef - read_write_flag: " << packet_struct.read_write_flag << std::endl;
		//~ std::cout << "packetdef - start_address: " << packet_struct.start_address << std::endl;
		//~ std::cout << "packetdef - data_size: " << packet_struct.data_size << std::endl;
		//~ std::cout << "packetdef - data_to_write: " << packet_struct.data_to_write[3] << std::endl;
	
		char packet[sizeof(fpga_request_write_packet_struct)];
		memcpy(packet, &packet_struct, sizeof(packet));	 
		
		zmq::message_t message(sizeof(packet));
		memcpy(message.data(), packet, sizeof(packet));
			
		fpga_map_request_port.send(message);
		
	} else{
		fpga_request_read_packet_struct packet_struct = fpga_request_read_packet_struct();
		packet_struct.return_address = (uint32_t) getpid(); //get process pid
		packet_struct.request_number = request_num;
		packet_struct.read_write_flag = read_write;
		packet_struct.start_address = channel;	
		packet_struct.data_size = 0; 
		
		//~ std::cout << "packetdef - packet size: " << sizeof(fpga_request_read_packet_struct) << std::endl;
		//~ std::cout << "packetdef - return_address: " << packet_struct.return_address << std::endl;
		//~ std::cout << "packetdef - request_number: " << unsigned(packet_struct.request_number) << std::endl;
		//~ std::cout << "packetdef - read_write_flag: " << packet_struct.read_write_flag << std::endl;
		//~ std::cout << "packetdef - start_address: " << packet_struct.start_address << std::endl;
		//~ std::cout << "packetdef - data_size: " << packet_struct.data_size << std::endl;
		
		char packet[sizeof(fpga_request_read_packet_struct)];	
		memcpy(packet, &packet_struct, sizeof(packet));	 
		
		zmq::message_t message(sizeof(packet));
		memcpy(message.data(), packet, sizeof(packet));
			
		fpga_map_request_port.send(message);
	}	  
}

void send_packet_pat_health(zmq::socket_t& pat_health_port, char* data)
{
	pat_health_packet_struct packet_struct = pat_health_packet_struct();
	packet_struct.return_address = (uint32_t) getpid(); //get process pid
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

fpga_answer_struct receive_packet_fpga_map_answer(zmq::socket_t& fpga_map_answer_port, bool read_write)
{
	zmq::message_t message;
	fpga_map_answer_port.recv(message, zmq::recv_flags::none);
	
	if(read_write == WRITE){	
		char packet[sizeof(fpga_answer_write_packet_struct)];
		memcpy(packet, message.data(), message.size());
		
		fpga_answer_write_packet_struct packet_struct = fpga_answer_write_packet_struct(); //initialize
		memcpy(&packet_struct, packet, sizeof(packet));
		
		//~ std::cout << "packetdef - packet size: " << sizeof(fpga_answer_write_packet_struct) << std::endl;
		//~ std::cout << "packetdef - header: " << packet_struct.header << std::endl;
		//~ std::cout << "packetdef - return_address: " << packet_struct.return_address << std::endl;
		//~ std::cout << "packetdef - request_number: " << unsigned(packet_struct.request_number) << std::endl;
		//~ std::cout << "packetdef - combined_flag: " << packet_struct.combined_flag << std::endl;
		//~ std::cout << "packetdef - start_address: " << packet_struct.start_address << std::endl;
		//~ std::cout << "packetdef - data_size: " << packet_struct.data_size << std::endl;
	
		fpga_answer_struct return_struct = fpga_answer_struct();
		return_struct.return_address = packet_struct.return_address; 
		return_struct.request_number = packet_struct.request_number;
		return_struct.rw_flag = packet_struct.combined_flag & 0x01;
        return_struct.error_flag = (packet_struct.combined_flag & 0x02) >> 1;
		return_struct.start_address = packet_struct.start_address;	
		return_struct.data_to_read = 0;
		
		return return_struct;
		
	} else{
		char packet[sizeof(fpga_answer_read_packet_struct)];
		memcpy(packet, message.data(), message.size());
		
		fpga_answer_read_packet_struct packet_struct = fpga_answer_read_packet_struct(); //initialize
		memcpy(&packet_struct, packet, sizeof(packet));
		
		//~ std::cout << "packetdef - packet size: " << sizeof(fpga_answer_read_packet_struct) << std::endl;
		//~ std::cout << "packetdef - header: " << packet_struct.header << std::endl;
		//~ std::cout << "packetdef - return_address: " << packet_struct.return_address << std::endl;
		//~ std::cout << "packetdef - request_number: " << unsigned(packet_struct.request_number) << std::endl;
		//~ std::cout << "packetdef - combined_flag: " << packet_struct.combined_flag << std::endl;
		//~ std::cout << "packetdef - start_address: " << packet_struct.start_address << std::endl;
		//~ std::cout << "packetdef - data_size: " << packet_struct.data_size << std::endl;
		//~ std::cout << "packetdef - data_to_read: " << atoi(packet_struct.data_to_read) << std::endl;
	
		fpga_answer_struct return_struct = fpga_answer_struct();
		return_struct.return_address = packet_struct.return_address; 
		return_struct.request_number = packet_struct.request_number;
		return_struct.rw_flag = packet_struct.combined_flag & 0x01;
        return_struct.error_flag = (packet_struct.combined_flag & 0x02) >> 1;
		return_struct.start_address = packet_struct.start_address;	
		return_struct.data_to_read = atoi(packet_struct.data_to_read); 
		
		return return_struct;
	}
	

	
}

// TODO: parse_packet_rx_pat (shouldn't need to receive bus commands for basic operation...)

// Check FPGA write request
bool check_fpga_map_write_request(zmq::socket_t& fpga_map_answer_port, std::vector<zmq::pollitem_t>& poll_fpga_answer, uint16_t channel, uint8_t request_number)
{
	// Listen for FPGA answer:
	for(int i = 0; i < MAX_FPGA_RESPONSE_ATTEMPTS; i++){		
		zmq::poll(poll_fpga_answer.data(), 1, POLL_TIME_FPGA_RESPONSE); // when timeout_ms (the third argument here) is -1, then block until ready to receive (based on: https://ogbe.net/blog/zmq_helloworld.html)
		if(poll_fpga_answer[0].revents & ZMQ_POLLIN){
			// received something on the first (only) socket
			fpga_answer_struct write_ans_struct = receive_packet_fpga_map_answer(fpga_map_answer_port, WRITE);
			//make sure message is for PAT process:
			// std::cout << "In packetdef.cpp - check_fpga_map_write_request: Response Attempt = " << i << std::endl;
			// std::cout << "In packetdef.cpp - check_fpga_map_write_request: return_address (Tx) = " << (uint32_t) getpid() << std::endl;
			// std::cout << "In packetdef.cpp - check_fpga_map_write_request: return_address (Rx) = " << write_ans_struct.return_address << std::endl;
			// std::cout << "In packetdef.cpp - check_fpga_map_write_request: request_number (Tx) = " << unsigned(request_number) << std::endl;
			// std::cout << "In packetdef.cpp - check_fpga_map_write_request: request_number (Rx) = " << unsigned(write_ans_struct.request_number) << std::endl;
			// std::cout << "In packetdef.cpp - check_fpga_map_write_request: start_address (Tx) = " << channel << std::endl;
			// std::cout << "In packetdef.cpp - check_fpga_map_write_request: start_address (Rx) = " << write_ans_struct.start_address << std::endl;
			// std::cout << "In packetdef.cpp - check_fpga_map_write_request: error_flag = " << write_ans_struct.error_flag << std::endl;
			if((((uint32_t) getpid()) == write_ans_struct.return_address) &&
				(request_number == write_ans_struct.request_number) && 
				(channel == write_ans_struct.start_address)){
				return !write_ans_struct.error_flag;
			} 
		}
	}
	// std::cout << "In packetdef.cpp - check_fpga_map_write_request: FPGA MAP Check Timeout!" << std::endl; 
	return false; //timeout
}

// Check FPGA map value via read request and answer
bool check_fpga_map_value(zmq::socket_t& fpga_map_answer_port, std::vector<zmq::pollitem_t>& poll_fpga_answer, zmq::socket_t& fpga_map_request_port, uint16_t channel, uint8_t data, uint8_t request_num)
{
	send_packet_fpga_map_request(fpga_map_request_port, channel, data, READ, request_num);

	// Listen for FPGA answer:
	for(int i = 0; i < MAX_FPGA_RESPONSE_ATTEMPTS; i++){		
		zmq::poll(poll_fpga_answer.data(), 1, POLL_TIME_FPGA_RESPONSE); // when timeout_ms (the third argument here) is -1, then block until ready to receive (based on: https://ogbe.net/blog/zmq_helloworld.html)
		if(poll_fpga_answer[0].revents & ZMQ_POLLIN){
			// received something on the first (only) socket
			fpga_answer_struct read_ans_struct = receive_packet_fpga_map_answer(fpga_map_answer_port, READ);
			//make sure message is for PAT process:
			std::cout << "In packetdef.cpp - check_fpga_map_value: Response Attempt = " << i << std::endl;
			std::cout << "In packetdef.cpp - check_fpga_map_value: return_address (Tx) = " << (uint32_t) getpid() << std::endl;
			std::cout << "In packetdef.cpp - check_fpga_map_value: return_address (Rx) = " << read_ans_struct.return_address << std::endl;
			std::cout << "In packetdef.cpp - check_fpga_map_value: request_number (Tx) = " << unsigned(request_number) << std::endl;
			std::cout << "In packetdef.cpp - check_fpga_map_value: request_number (Rx) = " << unsigned(read_ans_struct.request_number) << std::endl;
			std::cout << "In packetdef.cpp - check_fpga_map_value: start_address (Tx) = " << channel << std::endl;
			std::cout << "In packetdef.cpp - check_fpga_map_value: start_address (Rx) = " << read_ans_struct.start_address << std::endl;
			std::cout << "In packetdef.cpp - check_fpga_map_value: error_flag = " << read_ans_struct.error_flag << std::endl;
			std::cout << "In packetdef.cpp - check_fpga_map_value: data_to_read = " << read_ans_struct.data_to_read << std::endl;
			if((((uint32_t) getpid()) == read_ans_struct.return_address) &&
				(request_number == read_ans_struct.request_number) && 
				(channel == read_ans_struct.start_address) &&
				(data == read_ans_struct.data_to_read)){
				return !read_ans_struct.error_flag;
			} 
		}
	}
	// std::cout << "In packetdef.cpp - check_fpga_map_write_request: FPGA MAP Check Timeout!" << std::endl; 
	return false; //timeout
}