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
    
    int packet_buffer_length = 512;
    char packet_pat_health[packet_buffer_length];
    receive_packet(pat_health_port, packet_pat_health, packet_buffer_length);
    
    printf("%d \n", packet_pat_health);
    
    
    pat_health_packet_struct packet_pat_health_struct = pat_health_packet_struct();
		memcpy(&packet_pat_health_struct, packet_pat_health, sizeof(packet_pat_health));
		printf("Return Address: %d; Size: %d; Data: %s \n", packet_pat_health_struct.return_address, packet_pat_health_struct.data_size, packet_pat_health_struct.data_to_write);
    
    
    //pat_health_packet_struct *packet_struct = (pat_health_packet_struct *) pat_health_packet;
    //int health_data_array_size = packet_struct->data_size / sizeof(char);
    //std::string health_data_str = convertToString(packet_struct->data_to_write, health_data_array_size);
    //std::cout << health_data_str << std::endl; 
    //for (int i = 0; i < health_data_array_size; i++) { 
    //    std::cout << packet_struct->data_to_write; //[i];
    //} 
    //std::cout << std::endl;
    
    
    /*
        std::string pat_health_str = receive_packet(pat_health_port);
    std::cout << "Received: " << pat_health_str << std::endl;
    */
    
    /*
    // incoming pat_control packets
    zmq::message_t pat_health;

    // receive something on the pat_control_socket
    pat_health_port.recv(pat_health, zmq::recv_flags::none);
    std::string pat_health_str;
    pat_health_str.assign(static_cast<char *>(pat_health.data()), pat_health.size());
    std::cout << "Received: " << pat_health_str << std::endl;
    */
  }

  return 0;
}
