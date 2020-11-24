// set character buffer (pubsetbuf)
#include "packetdef.h"
#include <iostream>
#include <thread>
#include <chrono>

// converts character array 
// to string and returns it 
std::string convertToString(char* a, int size) 
{ 
    int i; 
    std::string s = ""; 
    for (i = 0; i < size; i++) { 
        s = s + a[i]; 
    } 
    return s; 
}

int main () {
  std::string PAT_HEALTH_PORT = "tcp://localhost:5559"; //SUB to Housekeeping
  std::string FPGA_MAP_REQUEST_PORT = "tcp://localhost:5558"; //SUB to FPGA Requests

  // initialize the zmq context with 1 IO threads 
  zmq::context_t context{1}; 
  
  // create the PAT_CONTROL_PORT SUB socket
  zmq::socket_t pat_health_port(context, ZMQ_SUB); 
  pat_health_port.connect(PAT_HEALTH_PORT); // connect to the transport
  pat_health_port.set(zmq::sockopt::subscribe, ""); // set the socket options such that we receive all messages. we can set filters here. this "filter" ("" and 0) subscribes to all messages.	
  std::cout << "Listening to PAT_HEALTH_PORT..." << std::endl; 
  
  // create the FPGA_MAP_REQUEST_PORT SUB socket
  zmq::socket_t fpga_map_request_port(context, ZMQ_SUB); 
  fpga_map_request_port.connect(FPGA_MAP_REQUEST_PORT); // connect to the transport
  fpga_map_request_port.set(zmq::sockopt::subscribe, ""); // set the socket options such that we receive all messages. we can set filters here. this "filter" ("" and 0) subscribes to all messages.	
  std::cout << "Listening to FPGA_MAP_REQUEST_PORT..." << std::endl; 
  
  for(;;){
    
    //Receive PAT Health Packet
    char pat_health_packet[BUFFER_SIZE];
    receive_packet(pat_health_port, pat_health_packet);
    
    pat_health_packet_struct packet_struct_pat_health = pat_health_packet_struct();
		memcpy(&packet_struct_pat_health, pat_health_packet, sizeof(pat_health_packet));
		printf("\nPAT Health Packet Received: \nReturn Address: %d \nSize: %d \nData: %s", packet_struct_pat_health.return_address, packet_struct_pat_health.data_size, packet_struct_pat_health.data_to_write);
    
    //Receive FPGA Request Packet
    char fpga_request_packet[BUFFER_SIZE];
    receive_packet(fpga_map_request_port, fpga_request_packet);
    
    fpga_request_packet_struct packet_struct_fpga_request = fpga_request_packet_struct();
		memcpy(&packet_struct_fpga_request, fpga_request_packet, sizeof(fpga_request_packet));
    printf("\nFPGA Map Request Packet Received:");
		printf("\nReturn Address: %d", packet_struct_fpga_request.return_address);
    printf("\nRequest Number: %d", packet_struct_fpga_request.request_number);
    printf("\nRead/Write Flag: %d", packet_struct_fpga_request.read_write_flag);
    printf("\nStart Address: %X", packet_struct_fpga_request.start_address);
    printf("\nData Size: %d", packet_struct_fpga_request.data_size);
    printf("\nData: %X", packet_struct_fpga_request.data_to_write);    
  }

  return 0;
}
