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

  // initialize the zmq context with 1 IO threads 
  zmq::context_t context{1}; 
  
  // create the PAT_CONTROL_PORT SUB socket
  zmq::socket_t pat_health_port(context, ZMQ_SUB); 
  pat_health_port.connect(PAT_HEALTH_PORT); // bind to the transport
  pat_health_port.set(zmq::sockopt::subscribe, ""); // set the socket options such that we receive all messages. we can set filters here. this "filter" ("" and 0) subscribes to all messages.	
  std::cout << "Listening to PAT_HEALTH_PORT..." << std::endl; 
  
  for(;;){
    
    char pat_health_packet[BUFFER_SIZE];
    receive_packet(pat_health_port, pat_health_packet);
    
    pat_health_packet_struct packet_pat_health_struct = pat_health_packet_struct();
		memcpy(&packet_pat_health_struct, pat_health_packet, sizeof(pat_health_packet));
		printf("Receiver - Return Address: %d; Size: %d; Data: %s \n", packet_pat_health_struct.return_address, packet_pat_health_struct.data_size, packet_pat_health_struct.data_to_write);
    
  }

  return 0;
}
