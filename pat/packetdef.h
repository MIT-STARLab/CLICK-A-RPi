// Author: Peter Grenfell
#ifndef __PACKETDEF
#define __PACKETDEF

#include <zmq.hpp>
#include <iostream>
#include <arpa/inet.h>
#include <sys/types.h>
#include <unistd.h>
#include <chrono>
#include <thread>

#define FRC_CH 0x35 //test channel to check fpga comms (FRC is free running counter, which is the number of clock cycles since last reset)
#define MAX_FPGA_RESPONSE_ATTEMPTS 10 //number of messages to look through on the FPGA answer port when checking a sent message
#define POLL_TIME_FPGA_RESPONSE 10 //milliseconds, time to wait between each poll of the FPGA answer port
#define TX_ADCS_APID 0x250 //(CLICK-A CPU Software Architecture on Google Drive)
#define TX_SELF_TEST_APID 0x3D2 //(CLICK-A CPU Software Architecture on Google Drive)
#define BUFFER_SIZE 256 //Needs to be long enough to fit all messages (I think longest one is 132)
#define WRITE 1 //(CLICK-A CPU Software Architecture on Google Drive)
#define READ 0 //(CLICK-A CPU Software Architecture on Google Drive)
#define FPGA_READ_SIZE 4
#define CMD_PAYLOAD_SIZE 256
//command list:
#define CMD_START_PAT 0x00
#define CMD_START_PAT_OPEN_LOOP 0x01
#define CMD_START_PAT_STATIC_POINT 0x02
#define CMD_START_PAT_BUS_FEEDBACK 0x03
#define CMD_START_PAT_OPEN_LOOP_BUS_FEEDBACK 0x04
#define CMD_UPDATE_TX_OFFSET_X 0x05
#define CMD_UPDATE_TX_OFFSET_Y 0x06
#define CMD_END_PAT 0x07
#define CMD_GET_IMAGE 0x08
#define CMD_CALIB_TEST 0x09
#define CMD_CALIB_LASER_TEST 0x0A
#define CMD_FSM_TEST 0x0B
#define CMD_BCN_ALIGN 0x0C
#define CMD_TX_ALIGN 0x0D
#define CMD_UPDATE_FSM_X 0x0E
#define CMD_UPDATE_FSM_Y 0x0F
#define CMD_SELF_TEST 0x10
#define CMD_END_PROCESS 0x11
#define CMD_SET_BEACON_X 0x12
#define CMD_SET_BEACON_Y 0x13
#define CMD_SET_BEACON_WINDOW_SIZE 0x14
#define CMD_SET_BEACON_MAX_EXP 0x15 
//status flags:
#define STATUS_CAMERA_INIT 0x00
#define STATUS_STANDBY 0x01
#define STATUS_MAIN 0x02
//self test flags:
#define PASS_SELF_TEST 0xFF
#define FAIL_SELF_TEST 0x0F
#define NULL_SELF_TEST 0x00
//pat test flag: 
#define TEST_FLAG 0xFFFF
#define FLIGHT_FLAG 0xAAAA

// Packet Definitions
struct fpga_request_write_packet_struct{
	uint32_t return_address;
	uint8_t request_number;
	bool read_write_flag;
	uint16_t start_address;
	uint32_t data_size;
	uint8_t data_to_write[4]; //struct alignment forces this to be 4 bytes, so need to add pre-padding since only have 1 byte of significant data
};

struct fpga_request_read_packet_struct{
	uint32_t return_address;
	uint8_t request_number;
	bool read_write_flag;
	uint16_t start_address;
	uint32_t data_size;
};

struct pat_health_packet_struct{
	uint32_t return_address;
	uint32_t data_size;
	char data_to_write[BUFFER_SIZE];
};

struct pat_status_packet_struct{
	uint32_t return_address;
	uint32_t status_flag; 
};

struct pat_tx_adcs_packet_struct{
	uint16_t apid;
	uint16_t data_size;
	uint32_t data_to_write[3];
};

struct pat_control_packet_struct{
	uint16_t command;
	uint16_t data_size;
	char data_to_read[CMD_PAYLOAD_SIZE];
};

struct fpga_answer_write_packet_struct{
	uint32_t return_address;
	uint8_t request_number;
	bool combined_flag;
	uint16_t start_address;
	uint32_t data_size;
};

struct fpga_answer_read_packet_struct{
	uint32_t return_address;
	uint8_t request_number;
	bool combined_flag;
	uint16_t start_address;
	uint32_t data_size;
	char data_to_read[FPGA_READ_SIZE]; //struct alignment forces this to be 4 bytes, so need to add pre-padding since only have 1 byte of significant data
};

struct fpga_answer_struct{
	uint32_t return_address;
	uint8_t request_number;
	bool rw_flag;
	bool error_flag;
	uint16_t start_address;
	uint8_t data_to_read;
};

struct pat_self_test_packet_struct{
	uint16_t apid;
	uint16_t data_size;
	uint8_t camera_test_result;
	uint8_t fpga_test_result;
	uint8_t laser_test_result;
	uint8_t fsm_test_result;
	uint8_t calibration_test_result;
	char error[BUFFER_SIZE];
};

// Packet Sending for PUB Processes:
void send_packet_fpga_map_request(zmq::socket_t& fpga_map_request_port, bool read_write, uint8_t request_num, uint16_t channel, uint8_t data = 0);

void send_packet_pat_health(zmq::socket_t& pat_health_port, char* data);

void send_packet_pat_status(zmq::socket_t& pat_status_port, uint32_t status);

void send_packet_tx_adcs(zmq::socket_t& tx_packets_port, float body_frame_x_angular_error_radians, float body_frame_y_angular_error_radians);

// Packet Receiving & Parsing for SUB Processes:
uint16_t receive_packet_pat_control(zmq::socket_t& pat_control_port, char* data_to_read = NULL);

fpga_answer_struct receive_packet_fpga_map_answer(zmq::socket_t& fpga_map_answer_port, bool read_write);

bool check_fpga_map_write_request(zmq::socket_t& fpga_map_answer_port, std::vector<zmq::pollitem_t>& poll_fpga_answer, uint16_t channel, uint8_t request_number);

bool check_fpga_map_value(zmq::socket_t& fpga_map_answer_port, std::vector<zmq::pollitem_t>& poll_fpga_answer, zmq::socket_t& fpga_map_request_port, uint16_t channel, uint8_t data, uint8_t request_num);

bool check_fpga_comms(zmq::socket_t& fpga_map_answer_port, std::vector<zmq::pollitem_t>& poll_fpga_answer, zmq::socket_t& fpga_map_request_port);

void send_packet_self_test(zmq::socket_t& tx_packets_port, uint8_t camera_test_result, uint8_t fpga_test_result, uint8_t laser_test_result, uint8_t fsm_test_result, uint8_t calibration_test_result, char* error);

// Optional: receive_packet_pat_rx (commands from bus)

#endif
