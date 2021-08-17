#!/usr/bin/env python
CHECK_ASSERTS = 1

COMMAND_HANDLERS_COUNT = 3
MESSAGE_TIMEOUT = 5000 # wait 5 seconds for ZeroMQ response

# File management
SYMLINK_MAX = 100 #Limits the number of files in /root/log/self_test and /root/log/laser_self_test
FILE_MGMT_MAX = 100 #Limits the number of files in /root/file_staging
#Note: See pat/log.h for file limit on /root/log/pat

#Default Seed register settings
DEFAULT_TEC_MSB = 5
DEFAULT_TEC_LSB = 114
DEFAULT_LD_MSB = 14
DEFAULT_LD_LSB = 33

DEFAULT_FTEC_MSB = 5
DEFAULT_FTEC_LSB = 67
DEFAULT_FLD_MSB = 14
DEFAULT_FLD_LSB = 33

DEFAULT_CW_TEC_MSB = 5
DEFAULT_CW_TEC_LSB = 151

DEFAULT_CW_FTEC_MSB = 5
DEFAULT_CW_FTEC_LSB = 107

#HIGH/LOW for Payload/flatsat
PPM4_THRESHOLDS = [-2.1, -3.0, -5,-7]

#HIGH/LOW for Payload/flatsat
CW_THRESHOLDS = [4.5, 3.6, -1,-2.3]

TRANSMIT_MESSAGE = "Hello I'm Mr. Meeseeks!"
TRANSMIT_TIME = 900

TRANSMIT_PPM = 16
#Payload = 1, flatsat = 0
SEED_SETTING = 1

#IPC Port Numbers
TEST_RESPONSE_PORT = "5599"

CH_HEARTBEAT_PORT = "5555"
HK_CONTROL_PORT = "5556"

FPGA_MAP_ANSWER_PORT = "5557"
FPGA_MAP_REQUEST_PORT = "5558"

PAT_HEALTH_PORT = "5559"
PAT_CONTROL_PORT = "5560"
PAT_STATUS_PORT = "5564"

TX_PACKETS_PORT = "5561"
RX_CMD_PACKETS_PORT = "5562"
RX_PAT_PACKETS_PORT = "5563"

LOAD_BALANCER_PORT = "5565"
LB_HEARTBEAT_PORT = "5566"

#FPGA interface
IPC_USES_SPI = 1
FPGA_SHELL_USES_IPC = 0
FPGA_SHELL_USES_SPI = 1

SPI_FREQ = 1000000
USB_DEVICE_ID = "1d50:602b:0002"

EDFA_READ_WRITE_DELAY = 0.1
EDFA_TIMOUT = 1.0
EDFA_VIRTUAL_REGS_GOOD_FOR = 0.5

#Time at tone packet APID
APID_TIME_AT_TONE = 0x280

PAT_MODE_FILENAME = "/root/lib/patMode.txt"

#Ground Command IDs
CMD_PL_REBOOT = 0x01
CMD_PL_SHUTDOWN = 0x02
CMD_PL_ENABLE_TIME = 0xC2
CMD_PL_EMERGENCY_REVERT = 0x5E
CMD_PL_EXEC_FILE = 0x67
CMD_PL_LIST_FILE = 0xFE
CMD_PL_AUTO_DOWNLINK_FILE = 0xAB
CMD_PL_ZIP_DOWNLINK_FILE = 0xAF
CMD_PL_ZIP_DOWNLINK_PAT_DATA = 0xAC
CMD_PL_DISASSEMBLE_FILE = 0x15
CMD_PL_REQUEST_FILE = 0x16
CMD_PL_UPLINK_FILE = 0xCD
CMD_PL_ASSEMBLE_FILE = 0x39
CMD_PL_VALIDATE_FILE = 0x40
CMD_PL_MOVE_FILE = 0x41
CMD_PL_DELETE_FILE = 0x42
CMD_PL_UNZIP_FILE = 0x43
CMD_PL_AUTO_ASSEMBLE_FILE = 0xCC
CMD_PL_UPDATE_OPTIONS = 0x2A
CMD_PL_SET_PAT_MODE = 0xB3
CMD_PL_UPDATE_PAT_OFFSET_PARAMS = 0xB4
CMD_PL_SINGLE_CAPTURE = 0xF1
CMD_PL_CALIB_LASER_TEST = 0x4C
CMD_PL_FSM_TEST = 0x28
CMD_PL_RUN_CALIBRATION = 0x32
CMD_PL_TEST_ADCS_FEEDBACK = 0x35
CMD_PL_UPDATE_ACQUISITION_PARAMS = 0x86
CMD_PL_TX_ALIGN = 0x87 #
CMD_PL_UPDATE_TX_OFFSETS = 0x88
CMD_PL_UPDATE_FSM_ANGLES = 0x89
CMD_PL_ENTER_PAT_MAIN = 0x90
CMD_PL_EXIT_PAT_MAIN = 0x91
CMD_PL_END_PAT_PROCESS = 0x92
CMD_PL_SET_FPGA = 0x54
CMD_PL_GET_FPGA = 0x0E
CMD_PL_SET_HK = 0x97
CMD_PL_ECHO = 0x3D
CMD_PL_NOOP = 0x5B
CMD_PL_SELF_TEST = 0x80
CMD_PL_UPDATE_SEED_PARAMS = 0x18
CMD_PL_DWNLINK_MODE = 0xE0 #Do not change - BCT
CMD_PL_DEBUG_MODE = 0xD0 #Do not change - BCT

#Telemetry APIDs
TLM_HK_SYS = 0x312
TLM_HK_PAT = 0x313
TLM_HK_FPGA_MAP = 0x314
TLM_HK_CH = 0x315
TLM_DL_FILE = 0x3E1
TLM_LIST_FILE = 0x3E0
TLM_DISASSEMBLE_FILE = 0x3E2
TLM_ASSEMBLE_FILE = 0x3E3
TLM_GET_FPGA = 0x3C0
TLM_ECHO = 0x3FF

#Self Test IDs
GENERAL_SELF_TEST = 0x00
LASER_SELF_TEST = 0x01
PAT_SELF_TEST = 0x02
THERMAL_SELF_TEST = 0x03
ALL_SELF_TEST = 0x04

#PAT IPC Command IDs [Shared Command Parameters with c-code (packetdef.h)]
PAT_CMD_PAYLOAD_SIZE = 256 #Only a fixed size is allowed in the C++ code (packetdef.h): add padding if necessary
PAT_CMD_START_PAT = 0x00
PAT_CMD_START_PAT_OPEN_LOOP = 0x01
PAT_CMD_START_PAT_STATIC_POINT = 0x02
PAT_CMD_START_PAT_BUS_FEEDBACK = 0x03
PAT_CMD_START_PAT_OPEN_LOOP_BUS_FEEDBACK = 0x04
PAT_CMD_UPDATE_TX_OFFSET_X = 0x05
PAT_CMD_UPDATE_TX_OFFSET_Y = 0x06
PAT_CMD_END_PAT = 0x07
PAT_CMD_GET_IMAGE = 0x08
PAT_CMD_CALIB_TEST = 0x09
PAT_CMD_CALIB_LASER_TEST = 0x0A
PAT_CMD_FSM_TEST = 0x0B
PAT_CMD_BCN_ALIGN = 0x0C
PAT_CMD_TX_ALIGN = 0x0D
PAT_CMD_UPDATE_FSM_X = 0x0E
PAT_CMD_UPDATE_FSM_Y = 0x0F
PAT_CMD_SELF_TEST = 0x10
PAT_CMD_END_PROCESS = 0x11
PAT_CMD_SET_BEACON_X = 0x12
PAT_CMD_SET_BEACON_Y = 0x13
PAT_CMD_SET_BEACON_WINDOW_SIZE = 0x14
PAT_CMD_SET_BEACON_MAX_EXP = 0x15
PAT_CMD_SET_GET_IMAGE_CENTER_X = 0x16
PAT_CMD_SET_GET_IMAGE_CENTER_Y = 0x17
PAT_CMD_SET_GET_IMAGE_WINDOW_WIDTH = 0x18
PAT_CMD_SET_GET_IMAGE_WINDOW_HEIGHT = 0x19
PAT_CMD_UPDATE_PERIOD_CALCULATE_TX_OFFSET = 0x1A
PAT_CMD_ENABLE_DITHER_TX_OFFSET = 0x1B
PAT_CMD_UPDATE_PERIOD_DITHER_TX_OFFSET = 0x1C
PAT_CMD_TEST_BUS_FEEDBACK = 0x1D
#PAT Status Flags
PAT_STATUS_CAMERA_INIT = 0x00
PAT_STATUS_STANDBY = 0x01
PAT_STATUS_STANDBY_CALIBRATED = 0x02
PAT_STATUS_STANDBY_SELF_TEST_PASSED = 0x03
PAT_STATUS_STANDBY_SELF_TEST_FAILED = 0x04
PAT_STATUS_MAIN = 0x05
#PAT Main Mode Entry Flag
PAT_SKIP_CALIB_FLAG = 0xFFFF
PAT_DO_CALIB_FLAG = 0xAAAA
PAT_ENABLE_DITHER = 0xFF
#Camera Params
CAMERA_WIDTH = 2592
CAMERA_HEIGHT = 1944
CAMERA_MAX_EXP = 10000000
CAMERA_MIN_EXP = 10
#Default Data in offsetParams.csv
DEFAULT_DATA_OFFSET_PARAMS = [['PERIOD_CALCULATE_TX_OFFSETS', ' 1000.0'], ['PERIOD_DITHER_TX_OFFSETS', ' 10.0'], ['TX_OFFSET_X_DEFAULT', ' -20'], ['TX_OFFSET_Y_DEFAULT', ' 120'], ['TX_OFFSET_SLOPE_X', ' 0.34619'], ['TX_OFFSET_BIAS_X', ' -29.464'], ['TX_OFFSET_QUADRATIC_Y', ' 0.0084797'], ['TX_OFFSET_SLOPE_Y', ' -0.34707'], ['TX_OFFSET_BIAS_Y', ' 123.46'], ['TX_OFFSET_DITHER_X_RADIUS', ' 4.0'], ['TX_OFFSET_DITHER_Y_RADIUS', ' 1.0'], ['DITHER_COUNT_PERIOD', ' 10']]  

#Calibration Laser DAC setting
CAL_LASER_DAC_SETTING = 6700

#Timout Heat to 0C
TIMEOUT_HEAT_TO_0C = 1200 #seconds
OVERRIDE_HEAT_TO_0C = False #option to override heating to 0C if RTDs break or something and we want to test anyways

# HK Options Settings
HK_FPGA_REQ_ENABLE = 1
HK_SYS_HK_SEND_ENABLE = 1
HK_PAT_HK_SEND_ENABLE = 1
HK_CH_HK_SEND_ENABLE = 1
HK_CH_RESTART_ENABLE = 1
HK_PAT_RESTART_ENABLE = 1
HK_FPGA_RESTART_ENABLE = 1
HK_LB_RESTART_ENABLE = 1
HK_ALLPKTS_SEND_ENABLE = 1

HK_FPGA_CHECK_PD = 5 #seconds, default period of hb checking
HK_SYS_CHECK_PD = 1 #seconds, default period of hb checking
HK_CH_CHECK_PD = 10 #seconds, default period of hb checking
HK_LB_CHECK_PD = 10 #seconds, default period of hb checking
HK_PAT_CHECK_PD = 10 #seconds, default period of hb checking
HK_FPGA_CHECK_PD_MIN = 1 #seconds, minimum period of hb checking
HK_SYS_CHECK_PD_MIN = 1 #seconds, minimum period of hb checking
HK_CH_CHECK_PD_MIN = 2 #seconds, minimum period of hb checking
HK_LB_CHECK_PD_MIN = 2 #seconds, minimum period of hb checking
HK_PAT_CHECK_PD_MIN = 2 #seconds, minimum period of hb checking
HK_CH_HEARTBEAT_PD = 0.5 #seconds, period of hb sending
HK_LB_HEARTBEAT_PD = 0.5 #seconds, period of hb sending

# File Handling Options Settings
FL_ERR_EMPTY_DIR = 0x01
FL_ERR_FILE_NAME = 0x02
FL_ERR_SEQ_LEN = 0x03
FL_ERR_MISSING_CHUNK = 0x04
FL_ERR_OUT_OF_BOUNDS = 0x05
FL_SUCCESS = 0xFF

# Set Time Flag
TIME_SET_ENABLE = 0

FPGA_TELEM_REGS = sum([range(0,5), range(32,39), range(47,49), range(53,55), [57], range(60,64), range(96,98), range(200,206), range(300,304), range(602,612), range(502,510)],[])
#FPGA_TELEM_REGS = sum([range(602,612)],[])
# For housekeeping/commandhandler interface
CMD_ACK = 0x0F
CMD_ERR = 0xF0

HK_CONTROL_ACK = 0x01
HK_CONTROL_LOG = 0x02
HK_CONTROL_CH = 0x03

#Error IDs
ERR_HK_RESTART = 0x380 #Change
ERR_FL_FILE_INVALID = 0x381 #Change
ERR_DPKT_CRC_INVALID = 0x382 #Change
ERR_GEN_EXCEPTION = 0x387

RAISE_ENABLE = 0
FULL_TRACE_ENABLE = 1

BUS_DATA_LEN = 4082
