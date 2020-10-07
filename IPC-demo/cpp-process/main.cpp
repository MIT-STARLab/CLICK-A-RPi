#include <string>
#include <chrono>
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
    socket.bind("tcp://*:5555");

    // prepare some static data for responses
    const std::string data{"11f51108f21ac0680ec5f356a4ab32cd4c4b7fd1ca1544a061622d2cffdc59bbd186e130d33239b07ff427d605d3eebeb50d58ab5f80b8bce9b53c5674c1012e908df84311b47b48b15848eaff325d880645c21286a6f30796e9191d4cb19f8e665cf394c0fa6603892ac9c94a3e784daeb1410ab17548ab9068c882452367122322531ba12d1b7d9b952a34bdb68cf5e9f6688bb1fe3322fca43a704eab864e136bc171d3c1065d42d28745fa37855c5c610b895fb001229e29be3bfdaed73d49f6ec9683f3afd2a9e680d2b3bcb77f898b3facacfc5448b654605aed123b3c08dea062cee7e176d288856b5f776b9abee9efa3d2a4aa9e795976244340817b16dfe66dae446aab5016cd087debd2e9becde90988c290356a1ddfc6dac9ce66b712bcdcae41d463596b12812f1cfb71cb0bc6dae803d8c970a9a85d0ee625a996e82541109a78d8c0394cf39f408925b135642d5182efaad0ef279a48e25c4cf10655b3b96fd2f422dc2c4b4f5ba250fcff2c87a15e0af767a84a642479ca116488a2ef6341cf2c97e22abd69b950b178dc0d68b84cb610fc53cec1c1d454b4f443897c3f61990f9bb61fcfe78d8d88e4067d7cbec745829f32a919b754aa845f43a806134b89579c1d4e74125b496c0b9fa001109d1f6d46455689f460ce2cae4098597522980713a14f93768f16d5b70de1b08d7e22d97fb05e82de385"};
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

        // send the reply to the client
        socket.send(zmq::buffer(data), zmq::send_flags::none);
    }

    return 0;
}
