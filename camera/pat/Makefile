## MatrixVision SDK
mvINC = /opt/mvIMPACT_Acquire/
mvLIB = /opt/mvIMPACT_Acquire/lib/armhf/

## nonflight, SocketIO SDK
#ioINC = /opt/socket.io-client-cpp/build/include/
#ioLIB = /opt/socket.io-client-cpp/build/lib/Release/

## Compiler/Linker config
CC = g++
CFLAGS = -O2 -std=c++11 -Wall -W -fPIC -D_REENTRANT -pedantic -Wno-format -Wno-unused-parameter -fvisibility=hidden
LIBS = -lmvPropHandling -lmvDeviceManager -lpthread -lboost_system
#nonflight: -lmvPropHandling -lmvDeviceManager -lsioclient -lpthread -lboost_system -lpigpiod_if2
## Output
ODIR = .
POINTING_OUT = pat
#DISTURBANCE_OUT = disturbance #nonflight
POINTING_OBJ = main.o camera.o processing.o fsm.o tracking.o calibration.o fpga.o EasyBMP.o #removed link.o, nonflight
#DISTURBANCE_OBJ = dist.o fsm.o #nonflight

all: $(POINTING_OUT) $(DISTURBANCE_OUT)

%.o: %.cpp
	$(CC) -c -o $@ $< $(CFLAGS) -I$(mvINC) #nonflight, removed -I$(ioINC)

$(POINTING_OUT): $(POINTING_OBJ)
	$(CC) -o $@ $^ $(CFLAGS) -L$(mvLIB) $(LIBS) #nonflight, removed -L$(ioLIB)

#nonflight
#$(DISTURBANCE_OUT): $(DISTURBANCE_OBJ)
#	$(CC) -o $@ $^ $(CFLAGS) -L$(mvLIB) -L$(ioLIB) $(LIBS)

debug: CFLAGS += -DDEBUG -g
debug: all

run:
	# ./$(OUT)
	scp main.cpp pi@10.0.5.2:/home/pi/closed_loop/

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o $(POINTING_OUT) #$(DISTURBANCE_OUT)
