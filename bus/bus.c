#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdbool.h>

#include "zhelpers.h"

typedef struct {
	int size;
	int start;
	int count;
	char **element;
} buffer_t;

void init(buffer_t *buffer, int size) {
	buffer->size = size;
	buffer->start = 0;
	buffer->count = 0;
	buffer->element = malloc(sizeof(buffer->element)*size);
}

bool full(buffer_t * buffer) {
	return buffer->count == buffer->size;
}

bool empty(buffer_t *buffer) {
	return buffer->count == 0;
}

void push(buffer_t *buffer, void *data) {
	int index;
	if (full(buffer)) {
		printf("Buffer full, pop elements first\n");
	} else {
		index = buffer->start + buffer->count++;
		if (index >= buffer->size) {
			index = 0;
		}
		buffer->element[index] = data;
	}
}

void * popqueue(buffer_t *buffer) {
	void * element;
	if (empty(buffer)) {
		printf("Buffer empty\n");
		return "0";
	} else {
		element = buffer->element[buffer->start];
		buffer->start++;
		buffer->count--;
		if (buffer->start == buffer->size) {
			buffer->start = 0;
		}
		return element;
	}
}

int main (int argc, char **argv) {

	int c;
	int fpga_reg = 0;
	int fpga_val = 0;

	while((c = getopt(argc, argv, "cfrw:")) != -1){
		switch(c) {
			case 'c':
				printf("not doing camera testing yet\n");
				break;
			case 'f':
				// pipe to fpga
				break;
			case 'r':
				fpga_reg = atoi(optarg);
				break;
			case 'w':
				fpga_val = atoi(optarg);
				break;
			default:
				printf("No argument provided, please consider revising your statement\n");
				break;
		}
	}

	printf("fpga value = %d\n", fpga_val);

	void *context = zmq_ctx_new();
	void *cameraReceiver = zmq_socket(context, ZMQ_PULL);
	int rc = zmq_connect(cameraReceiver, "tcp://localhost:5555");
	assert (rc == 0);

	void *fpgaSender = zmq_socket(context, ZMQ_PUSH);
	int rc1 = zmq_bind(fpgaSender, "tcp://*:5556");
	assert (rc1 == 0);

	void *fpgaReceiver = zmq_socket(context, ZMQ_PULL);
	int rc2 = zmq_connect(fpgaReceiver, "tcp://localhost:5557");
	assert (rc2 == 0);

	buffer_t buffer;
	init(&buffer, 15);

	char *command = (char *) malloc(256*sizeof(char));
	strcpy(command, "downlink poem_zone.txt");
	puts(command);
	s_send(fpgaSender, command);


	while(1) {
		char msg [64];
		zmq_pollitem_t items [] = {
			{ cameraReceiver,   0, ZMQ_POLLIN, 0 },
			{ fpgaReceiver, 0, ZMQ_POLLIN, 0 }
		};
		//poll every second
		zmq_poll (items, 2, 1000);
		if (items [0].revents & ZMQ_POLLIN) {
			int size = zmq_recv (cameraReceiver, msg, 63, 0);
			if (size != -1) {
				//push to circular buffer
				push(&buffer, msg);
			}
		}
		if (items [1].revents & ZMQ_POLLIN) {
			int size = zmq_recv (fpgaReceiver, msg, 63, 0);
			if (size != -1) {
        //Receive from fpga.py, do something
				puts(msg);
			}
		}
		memset(msg, 0, strlen(msg));
		/*
		RUN ON COMMAND FROM BUS TO SEND FILES TO FPGA
		popqueue(&buffer);
		s_send(fpgaSender, msg);
		*/

	}
	zmq_close(cameraReceiver);
	zmq_close(fpgaSender);
	zmq_close(fpgaReceiver);
	zmq_ctx_destroy(context);
	return 0;	
}
