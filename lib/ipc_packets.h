#include <cstdint>
#include <array>
#include <zmq.hpp>

struct tx_packet_struct {
	std::uint8_t APID;
	std::uint16_t size;
	std::
};


union tx_packet {
	
	struct tx_packet_struct {
		...
	};
	
	
	
}