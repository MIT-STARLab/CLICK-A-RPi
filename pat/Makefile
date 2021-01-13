## MatrixVision SDK
mvINC = /opt/mvIMPACT_Acquire/
mvLIB = /opt/mvIMPACT_Acquire/lib/armhf/
## Compiler/Linker config
CXX = g++
CXXFLAGS = -O2 -std=c++11 -Wall -W -fPIC -D_REENTRANT -pedantic -Wno-format -Wno-unused-parameter -fvisibility=hidden
LIBS = -lmvPropHandling -lmvDeviceManager -lpthread -lzmq
## Output
OBJ_DIR = ./build
BIN_DIR = ./bin
SOURCES = main.cpp camera.cpp processing.cpp fsm.cpp tracking.cpp calibration.cpp packetdef.cpp log.cpp export/EasyBMP.cpp export/lodepng.cpp
OBJECTS= $(SOURCES:%.cpp=$(OBJ_DIR)/%.o)

all: dir pat enumerate

dir:
	mkdir -p $(OBJ_DIR)/export
	mkdir -p $(BIN_DIR)

$(OBJ_DIR)/%.o: %.cpp
	$(CXX) -c -o $@ $< $(CXXFLAGS) -I$(mvINC) 

pat: $(OBJECTS)
	$(CXX) -o $(BIN_DIR)/$@ $^ $(CXXFLAGS) -L$(mvLIB) $(LIBS)

enumerate : $(OBJ_DIR)/enumerate.o
	$(CXX) -o $(BIN_DIR)/$@ $^ $(CXXFLAGS) -L$(mvLIB) -lmvDeviceManager

clean:
	rm -f $(OBJ_DIR) $(BIN_DIR)
