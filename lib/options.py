COMMAND_HANDLERS_COUNT = 1
MESSAGE_TIMEOUT = 5000 # wait 5 seconds for ZeroMQ response
CH_HEARTBEAT_PD = 6000

#IPC Port Numbers
TEST_RESPONSE_PORT = "5599"

CH_HEARTBEAT_PORT = "5555"
HK_CONTROL_PORT = "5556"

FPGA_MAP_ANSWER_PORT = "5557"
FPGA_MAP_REQUEST_PORT = "5558"

PAT_HEALTH_PORT = "5559"
PAT_CONTROL_PORT = "5560"

TX_PACKETS_PORT = "5561"
RX_CMD_PACKETS_PORT = "5562"
RX_PAT_PACKETS_PORT = "5563"

#Ground Command IDs
CMD_PL_REBOOT = 0x01
CMD_PL_ENABLE_TIME = 0xC2
CMD_PL_DISABLE_TIME = 0xA4
CMD_PL_EXEC_FILE = 0x67
CMD_PL_LIST_FILE = 0xFE
CMD_PL_REQUEST_FILE = 0xAB
CMD_PL_UPLOAD_FILE_TOSTAGING = 0xCD
CMD_PL_SET_PAT_MODE = 0xF0 #Temporary
CMD_PL_SINGLE_CAPTURE = 0xF1
CMD_PL_CALIB_LASER_TEST = 0xF2 #Temporary
CMD_PL_FSM_TEST = 0xF3 #Temporary
CMD_PL_RUN_CALIBRATION = 0x32
CMD_PL_SET_FPGA = 0x54
CMD_PL_GET_FPGA = 0x0E
CMD_PL_SET_HK = 0x97
CMD_PL_ECHO = 0x3D
CMD_PL_NOOP = 0x5B
CMD_PL_DWNLINK_MODE = 0x0500
CMD_PL_DEBUG_MODE = 0x0501

#Telemetry APIDs
TLM_DL_FILE = 0x387 #TBR
TLM_HK_CPU = 0x312 #TBR
TLM_HK_PAT = 0x313 #TBR
TLM_HK_FPGA_MAP = 0x314 #TBR
TLM_ECHO = 0x3FF 
TLM_LIST_FILE = 0x3E0 
TLM_GET_FPGA = 0x3C0

#PAT IPC Command IDs [Shared Command Parameters with c-code (packetdef.h)]
PAT_CMD_PAYLOAD_SIZE = 256 #Only a fixed size is allowed in the C++ code (packetdef.h): add padding if necessary
PAT_CMD_HEADER_SIZE = 5 #Only a fixed size is allowed in the C++ code (packetdef.h): add padding if necessary
PAT_CMD_START_PAT_OPEN_LOOP = 0x01
PAT_CMD_START_PAT_STATIC_POINT = 0x02
PAT_CMD_START_PAT_BUS_FEEDBACK = 0x03
PAT_CMD_START_PAT = 0x04
PAT_CMD_END_PAT = 0x05
PAT_CMD_GET_IMAGE = 0x06
PAT_CMD_CALIB_TEST = 0x07
PAT_CMD_CALIB_LASER_TEST = 0x08
PAT_CMD_FSM_TEST = 0x09