#include <string>
#include <chrono>
#include <ctime>
#include <thread>
#include <iostream>

#include <zmq.hpp>

int main()
{
    using namespace std::chrono_literals;

    // initialize the zmq context with a single IO thread
    zmq::context_t context{1};

    // construct a REP (reply) socket and bind to interface
    zmq::socket_t socket{context, zmq::socket_type::rep};
    socket.bind("tcp://*:5557");


    // std::string new_n;

    for (;;)
    {

        //generate random number (1-100)
        // new_n = std::to_string ((rand() % 100) + 1);

        zmq::message_t request;

        // receive a request from client
        socket.recv(request, zmq::recv_flags::none);
        // std::cout << "Received " << request.to_string() << std::endl;

        // simulate work
        //std::this_thread::sleep_for(1s);

        std::time_t t = std::time(0);
        std::string timestamp = std::to_string(t);
        std::string data{timestamp};

        // send the reply to the client
        socket.send(zmq::buffer(data), zmq::send_flags::none);
    }

    return 0;
}
