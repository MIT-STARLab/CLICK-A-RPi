'''

This is a alias map for fpga map requests.
Please use all human readable names in the code and no individual register values.

All blocks are specified as (starting reg address, length of block)

Human readable value block starts at reg 200;

'''
import math
import collections 

CTL = 0
CTL_TX_EN      = 0b00000001
CTL_SPI2_EN    = 0b00000100
CTL_STALL      = 0b00001000
CTL_ERX_FIFO   = 0b00010000

LTSa = 1
LTSb = 2
LBCa = 3
LBCb = 4

LTRa = 96
LTRb = 97

THRa = 5
THRb = 6
THRc = 7
FSMa = 8
FSMb = 9
FSMc = 10

DATA = 13

CAL = 32 #Cal laser
PO1 = 33 #LD Bias
PO2 = 34 #EDFA
PO3 = 35 #Heaters
PO4 = 36 #TEC
HE1 = 37 #Heater 1
HE2 = 38 #Heater 2

FLG = 63
FLG_DCM_LOCKED = 0b00000001
FLG_CRC_ERROR  = 0b00010000
FLG_RX_NEW     = 0b00100000
FLG_TX_READY   = 0b01000000
FLG_MOD_ERROR  = 0b10000000

# EDFA UART
ETX = 11
ERX = 65
ERF = 66
EFL = 67
EFL_EMPTY      = 0b00000010

# BIST capture
SCF = 15 # BIST capture flags
SCF_CAPTURE_DONE   = 0b001
SCF_PLL_LOCKED     = 0b010
SCF_PLL_RECONF_RDY = 0b100

SCTa = 16 # Total, LSB
SCTb = 17 # Total, MSB
SCAa = 18
SCAb = 19
SCBa = 20
SCBb = 21
SCCa = 22
SCCb = 23

SCN = 24
SCN_RUN_CAPTURE     = 0b00001
SCN_PROG_PLL        = 0b00010
SCN_SHUTDOWN_PLL    = 0b00100
SCN_BYPASS_PLL      = 0b01000
SCN_PLL_PAGE_SELECT = 0b10000
SCP = 25
SCD = 26
SPL = 27
SPH = 28


REGISTER_TYPE = collections.defaultdict(lambda : 'xxxx')
BYTE_1 = {}
BYTE_2 = {}
BYTE_3 = {}
BYTE_4 = {}

# ----------------- Base registers ----------------- 
for reg in range(  0, 128):
    REGISTER_TYPE[reg] = 'xxxB'
    
class Power:
    def __init__(self, handler):
        self.handler = handler
        
    def calib_diode_on(self):  self.handler.write_reg(CAL, 85)
    def calib_diode_off(self): self.handler.write_reg(CAL, 15)
    def bias_on(self):         self.handler.write_reg(PO1, 85)
    def bias_off(self):        self.handler.write_reg(PO1, 15)
    def edfa_on(self):         self.handler.write_reg(PO2, 85)
    def edfa_off(self):        self.handler.write_reg(PO2, 15)
    def heaters_on(self):      self.handler.write_reg(PO3, 85)
    def heaters_off(self):     self.handler.write_reg(PO3, 15)
    def tec_on(self):          self.handler.write_reg(PO4, 85)
    def tec_off(self):         self.handler.write_reg(PO4, 15)
    def heater_1_on(self):     self.handler.write_reg(HE1, 85)
    def heater_1_off(self):    self.handler.write_reg(HE1, 15)
    def heater_2_on(self):     self.handler.write_reg(HE2, 85)
    def heater_2_off(self):    self.handler.write_reg(HE2, 15)
    
# ----------------- Temperatures --------------------


TEMPERATURE_BLOCK = list(range(200, 206))
PD_TEMP, EDFA_TEMP, CAMERA_TEMP, TOSA_TEMP, LENS_TEMP, RACEWAY_TEMP = TEMPERATURE_BLOCK[0:6]
for reg in TEMPERATURE_BLOCK:
    REGISTER_TYPE[reg] = 'f'
    BYTE_1[reg] = 2*reg - 302    
    BYTE_2[reg] = 2*reg - 301
    
def decode_temperature(msb, lsb):
    # 12 bit ADC
    val = msb*256 + lsb # complete value
    Vadc = val*2.5/2**12 # volatge as a float
    
    # # Wheatstone bridge
    R = 920.0
    Vs = 3.3
    amp_gain = 8.5
    #Rrtd = (R*Vs - 2*R*Vadc) / (R*Vs + 2*R*Vadc) * R
    Rrtd = (-Vs*R/((Vadc/amp_gain) -Vs/2)) - R 
    # # RTD probe
    A =  3.81e-3
    B = -6.02e-7
    R0 = 1000.0
    temp = (-A + math.sqrt(A**2 - 4*B*(1-Rrtd/R0))) / (2*B)
    
    return temp
    
# ----------------- Current consumption ----------------- 


CURRENT_BLOCK = list(range(300, 304))
TEC_CURRENT, HEATER_CURRENT, EDFA_CURRENT, LD_CURRENT = CURRENT_BLOCK[0:4]
for reg in CURRENT_BLOCK:
    REGISTER_TYPE[reg] = 'f'
    BYTE_3[reg] = 2*reg - 488    
    BYTE_4[reg] = 2*reg - 487

#Converts adc value to amps according to CLICK-A FPGA current sensor schematic
def decode_current(msb, lsb):
    val = msb*256 + lsb # complete value
    Vadc = val*3.3/2**12 # voltage as a float
    current = Vadc/(101*.01) #amplifier output/gain/resistor = current

    return current

# ----------------- Seed Set Point ----------------- 
REGISTER_TYPE[400] = 'I'

# ----------------- Bias Current ----------------- 
REGISTER_TYPE[401] = 'I'

# ----------------- DACs ----------------- 
DAC_SETUP  = 499
DAC_ENABLE = 500
DAC_RESET  = 501
DAC_BIST_A = 0b00000001
DAC_BIST_B = 0b00000010
DAC_BIST_C = 0b00000100
DAC_BIST_D = 0b00001000
DAC_FSM_A  = 0b00010000
DAC_FSM_B  = 0b00100000
DAC_FSM_C  = 0b01000000
DAC_FSM_D  = 0b10000000
DAC_OEN    = 0b00 << 8
DAC_1K     = 0b01 << 8
DAC_100K   = 0b10 << 8
DAC_HIGHZ  = 0b11 << 8
REGISTER_TYPE[DAC_SETUP] = 'I'
REGISTER_TYPE[DAC_ENABLE] = 'xxH'
REGISTER_TYPE[DAC_RESET] = 'xxxB'


DAC_BLOCK = list(range(502, 510))
DAC_1_A, DAC_1_B, DAC_1_C, DAC_1_D = DAC_BLOCK[0:4]
DAC_2_A, DAC_2_B, DAC_2_C, DAC_2_D = DAC_BLOCK[0:4]
for reg in DAC_BLOCK:
    REGISTER_TYPE[reg] = 'xxH'
    
class DAC:
    def __init__(self, handler):
        self.handler = handler
        
    def reset_bist(self,is_por=1,ref_en=1):
        if is_por: is_por = 0b0100
        if ref_en: ref_en = 0b1000
        self.handler.write_reg(DAC_RESET, ref_en | is_por | 0b0001)
    
    def reset_fsm(self):
        if is_por: is_por = 0b0100
        if ref_en: ref_en = 0b1000
        self.handler.write_reg(DAC_RESET, ref_en | is_por | 0b0001)
    
    def enable_output(self,mask):
        self.handler.write_reg(DAC_ENABLE, DAC_OEN | mask)
        
    def disable_output(self,mask):
        self.handler.write_reg(DAC_ENABLE, DAC_HIGHZ | mask)
    
# ----------------- EDFA -----------------
EDFA_IN_STR    = 600
EDFA_OUT_STR   = 601
REGISTER_TYPE[EDFA_IN_STR]  = 'str'
REGISTER_TYPE[EDFA_OUT_STR] = 'str'

EDFA_PARSED_BLOCK = list(range(602, 612))
EDFA_EN_PIN   = 602
REGISTER_TYPE[EDFA_EN_PIN] = 'xxxB'
EDFA_MODE      = 603
EDFA_MODE_ACC  = 1
EDFA_MODE_AOPC = 2
EDFA_MODE_APC  = 3
REGISTER_TYPE[EDFA_MODE] = 'xxxB'
EDFA_DIODE_ON  = 604
REGISTER_TYPE[EDFA_DIODE_ON] = 'xxxB'
EDFA_MYSTERY_TEMP = 605
REGISTER_TYPE[EDFA_MYSTERY_TEMP] = 'f'
EDFA_POWER_IN  = 606
REGISTER_TYPE[EDFA_POWER_IN] = 'f'
EDFA_POWER_OUT = 607
REGISTER_TYPE[EDFA_POWER_OUT] = 'f'
EDFA_PRE_CURRENT = 608
REGISTER_TYPE[EDFA_PRE_CURRENT] = 'I'
EDFA_PRE_POWER = 609
REGISTER_TYPE[EDFA_PRE_POWER] = 'I'
EDFA_PUMP_CURRENT = 610
REGISTER_TYPE[EDFA_PUMP_CURRENT] = 'I'
EDFA_CASE_TEMP = 611
REGISTER_TYPE[EDFA_CASE_TEMP] = 'f'

   
class EDFA:
    def __init__(self, handler):
        self.handler = handler
        
    def write_string(self,tx_str):
        return self.handler.write_reg(EDFA_IN_STR,tx_str)
        
    def read_string(self):
        return self.handler.read_reg(EDFA_OUT_STR)
        
    def send_command(self,tx_str):
        self.handler.write_reg(EDFA_IN_STR,tx_str)
        res_str = self.handler.read_reg(EDFA_OUT_STR)
        if res_str[1] == 'S': return True
        else: return False
        
    def turn_on(self):
        return self.send_command('edfa on\n')
        
    def turn_off(self):
        return self.send_command('edfa off\n')
        
    def mode_aopc(self):
        return self.send_command('mode aopc\n')
        
    def mode_acc(self):
        return self.send_command('mode acc\n')
        
    def set_pump_current(self, current_mA):
        return self.send_command('ldc ba %d\n' % current_mA)
        
    def is_pin_high(self):
        return self.handler.read_reg(EDFA_EN_PIN)
        
    def get_mode(self):
        return self.handler.read_reg(EDFA_MODE)
        
    def is_pump_on(self):
        return self.handler.read_reg(EDFA_DIODE_ON)
        
    def get_mystery_temp(self):
        return self.handler.read_reg(EDFA_MYSTERY_TEMP)

    def get_input_power_dBm(self):
        return self.handler.read_reg(EDFA_POWER_IN)

    def get_output_power_dBm(self):
        return self.handler.read_reg(EDFA_POWER_OUT)

    def get_preamp_current(self):
        return self.handler.read_reg(EDFA_PRE_CURRENT)
 
    def get_preamp_power(self):
        return self.handler.read_reg(EDFA_PRE_POWER)
 
    def get_pump_current(self):
        return self.handler.read_reg(EDFA_PUMP_CURRENT)
 
    def get_case_temp(self):
        return self.handler.read_reg(EDFA_CASE_TEMP)

'''
REGISTERS = [None] * 300 # [encoding B = byte, I = unsigned int, ? = bool, [physical registers, least to most significant bits], rw flag]

POWER_MANAGEMENT_BLOCK = [32, 7]
REGISTERS[32] = ['B', [32], True] #Calibration diode
REGISTERS[33] = ['B', [33], True] #Power on/off LD BIAS
REGISTERS[34] = ['B', [34], True] #Power on/off EDFA
REGISTERS[35] = ['B', [35], True] #Power on/off HEATERS
REGISTERS[36] = ['B', [36], True] #LD TEC Power on/off
REGISTERS[37] = ['B', [37], True] #Heater 1 on/off
REGISTERS[38] = ['B', [38], True] #Heater 2 on/off

TEMPERATURE_BLOCK = [98, 12]
REGISTERS[98] = ['B', [98], False] #Temperature 1 (MSB)
REGISTERS[99] = ['B', [99], False] #Temperature 1 (LSB)
REGISTERS[100] = ['B', [100], False] #Temperature 2 (MSB)
REGISTERS[101] = ['B', [101], False] #Temperature 2 (LSB)
REGISTERS[102] = ['B', [102], False] #Temperature 3 (MSB)
REGISTERS[103] = ['B', [103], False] #Temperature 3 (LSB)
REGISTERS[104] = ['B', [104], False] #Temperature 4 (MSB)
REGISTERS[105] = ['B', [105], False] #Temperature 4 (LSB)
REGISTERS[106] = ['B', [106], False] #Temperature 5 (MSB)
REGISTERS[107] = ['B', [107], False] #Temperature 5 (LSB)
REGISTERS[108] = ['B', [108], False] #Temperature 6 (MSB)
REGISTERS[109] = ['B', [109], False] #Temperature 6 (LSB)

CURRENT_BLOCK = [110, 8]
REGISTERS[110] = ['B', [110], False] #Current consumption LD Bias (MSB)
REGISTERS[111] = ['B', [111], False] #Current consumption LD Bias (LSB)
REGISTERS[112] = ['B', [112], False] #Current consumption TEC (MSB)
REGISTERS[113] = ['B', [113], False] #Current consumption TEC (LSB)
REGISTERS[114] = ['B', [114], False] #Current consumption HEATER (MSB)
REGISTERS[115] = ['B', [115], False] #Current consumption HEATER (LSB)
REGISTERS[116] = ['B', [116], False] #Current consumption EDFA (MSB)
REGISTERS[117] = ['B', [117], False] #Current consumption EDFA (LSB)

TEMPERATURE_CONTROLLER_BLOCK = [1, 2]
REGISTERS[1] = ['B', [1], True] #Temp Set Point (MSB)
REGISTERS[2] = ['B', [2], True] #Temp Set Point (LSB)

CURRENT_CONTROLLER_BLOCK = [3, 2]
REGISTERS[3] = ['B', [3], True] #Bias Current (MSB)
REGISTERS[4] = ['B', [4], True] #Bias Current (LSB)

'''
#Human readable
'''

TEMPERATURE_BLOCK_C = [200, 6]
REGISTERS[200] = ['I', [99,98], False] #Temperature 1
REGISTERS[201] = ['I', [101,100], False] #Temperature 2
REGISTERS[202] = ['I', [103,102], False] #Temperature 3
REGISTERS[203] = ['I', [105,104], False] #Temperature 4
REGISTERS[204] = ['I', [107,106], False] #Temperature 5
REGISTERS[205] = ['I', [109,108], False] #Temperature 6

CURRENT_BLOCK_I = [206, 4]
REGISTERS[206] = ['I', [111,110], False] #Current consumption LD Bias
REGISTERS[207] = ['I', [113,112], False] #Current consumption TEC
REGISTERS[208] = ['I', [115,114], False] #Current consumption HEATER
REGISTERS[209] = ['I', [117,116], False] #Current consumption EDFA

POWER_MANAGEMENT_BLOCK_B = [210, 7]
REGISTERS[210] = ['?', [210], True] #Calibration diode
REGISTERS[211] = ['?', [211], True] #Power on/off LD BIAS
REGISTERS[212] = ['?', [212], True] #Power on/off EDFA
REGISTERS[213] = ['?', [213], True] #Power on/off HEATERS
REGISTERS[214] = ['?', [214], True] #LD TEC Power on/off
REGISTERS[215] = ['?', [215], True] #Heater 1 on/off
REGISTERS[216] = ['?', [216], True] #Heater 2 on/off

TEMPERATURE_CONTROLLER_BLOCK_C = [217, 1]
REGISTERS[217] = ['I', [2,1], True] #Temp Set Point

CURRENT_CONTROLLER_BLOCK_I = [218, 1]
REGISTERS[218] = ['I', [4,3], True] #Bias Current
'''