'''

This is a alias map for fpga map requests.
Please use all human readable names in the code and no individual register values.

All blocks are specified as (starting reg address, length of block)

Human readable value block starts at reg 200;

'''
import math

REGISTER_TYPE = {}
BYTE_1 = {}
BYTE_2 = {}
BYTE_3 = {}

# ----------------- Base registers ----------------- 
for reg in range(  0, 128):
    REGISTER_TYPE[reg] = 'xxxB'
    
    
# ----------------- Temperatures --------------------

TEMPERATURE_BLOCK = list(range(200, 206))
for reg in TEMPERATURE_BLOCK:
    REGISTER_TYPE[reg] = 'f'
    BYTE_1[reg] = 2*reg - 302    
    BYTE_2[reg] = 2*reg - 301
    
def decode_temperature(msb, lsb):
    # 12 bit ADC
    val = msb*256 + lsb # complete value
    Vadc = val*2.5/2**12 # volatge as a float
    
    # Wheatstone bridge
    R = 920.0
    Vs = 3.3
    Rrtd = (R*Vs - 2*R*Vadc) / (R*Vs + 2*R*Vadc) * R
    
    # RTD probe
    A =  3.81e-3
    B = -6.02e-7
    R0 = 1000.0
    Temp = (-A + math.sqrt(A**2 - 4*B*(1-Rrtd/R0))) / (2*B)
    
    return Temp
    
# ----------------- Current consumption ----------------- 
CURRENT_BLOCK = list(range(206, 210))
for reg in CURRENT_BLOCK:
    REGISTER_TYPE[reg] = 'f'

# ----------------- Temp Set Point ----------------- 
REGISTER_TYPE[217] = 'I'

# ----------------- Bias Current ----------------- 
REGISTER_TYPE[218] = 'I'

# ----------------- Thresholds ----------------- 
for reg in range(219, 223):
    REGISTER_TYPE[reg] = 'I'

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