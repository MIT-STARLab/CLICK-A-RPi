## MatrixVision SDK
mvINC = /opt/mvIMPACT_Acquire/
mvLIB = /opt/mvIMPACT_Acquire/lib/armhf/

## Compiler/Linker config
CC = g++
CFLAGS = -O2 -std=c++11 -Wall -W -fPIC -D_REENTRANT -pedantic -Wno-format -Wno-unused-parameter -fvisibility=hidden
LIBS = -lmvPropHandling -lmvDeviceManager -lpthread -lzmq #-lboost_system
## Output
ODIR = .
POINTING_OUT = pat
TESTIPC_OUT = testipc
POINTING_OBJ = main.o camera.o processing.o fsm.o tracking.o calibration.o EasyBMP.o packetdef.o
TESTIPC_OBJ = testIPC.o packetdef.o

all: $(POINTING_OUT) $(TESTIPC_OUT)

%.o: %.cpp
	$(CC) -c -o $@ $< $(CFLAGS) -I$(mvINC) 

$(POINTING_OUT): $(POINTING_OBJ)
	$(CC) -o $@ $^ $(CFLAGS) -L$(mvLIB) $(LIBS) 
	
$(TESTIPC_OUT): $(TESTIPC_OBJ)
	$(CC) -o $@ $^ $(CFLAGS) -L$(mvLIB) $(LIBS) 

debug: CFLAGS += -DDEBUG -g
debug: all

run:
	# ./$(OUT)
	scp main.cpp pi@10.0.5.2:/home/pi/closed_loop/

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o $(POINTING_OUT) $(TESTIPC_OUT)
