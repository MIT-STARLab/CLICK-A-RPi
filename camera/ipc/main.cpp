#include <string>
#include <chrono>
#include <ctime>
#include <thread>
#include <iostream>
#include <sstream>
#include <string.h>
#include <zmq.hpp>

// https://ogbe.net/blog/zmq_helloworld.html

int main()
{
    using namespace std::chrono_literals;

    // define ports for PUB/SUB (this process binds)
    std::string TX_PACKETS_PORT = "tcp://*:55610";
    std::string PAT_CONTROL_PORT = "tcp://*:55600";

    // initialize the zmq context with a single IO thread
    zmq::context_t context{1};

    // create the TX_PACKETS_PORT PUB socket
    zmq::socket_t tx_packets_socket(context, ZMQ_PUB);
    // we need to bind to the transport
    tx_packets_socket.bind(TX_PACKETS_PORT);

    // create the PAT_CONTROL SUB socket
    zmq::socket_t pat_control_socket(context, ZMQ_SUB);
    // we need to bind to the transport
    pat_control_socket.bind(PAT_CONTROL_PORT);
    // set the socket options such that we receive all messages. we can set
    // filters here. this "filter" ("" and 0) subscribes to all messages.
    // pat_control_socket.setsockopt(ZMQ_SUBSCRIBE, "", 0); // DEPRECIATED!
    pat_control_socket.set(zmq::sockopt::subscribe, "");


    // construct a REP (reply) socket and bind to interface
    //zmq::socket_t socket{context, zmq::socket_type::rep};
    //socket.bind("tcp://*:5557");


    // std::string new_n;

    for (;;)
    {

        // incoming pat_control packets
        zmq::message_t pat_control;

        // receive something on the pat_control_socket
        pat_control_socket.recv(pat_control, zmq::recv_flags::none);
        std::string pat_control_str;
        pat_control_str.assign(static_cast<char *>(pat_control.data()), pat_control.size());
        std::cout << "Received: " << pat_control_str << std::endl;

        // create a message
        std::stringstream s;
        s << '12345 {"command": "camera"} ' << i;
        auto msg = s.str();
        zmq::message_t tx_packets(msg.length());
        memcpy(tx_packets.data(), msg.c_str(), msg.length());
        publisher.send(tx_packets);

    }

    return 0;
}
