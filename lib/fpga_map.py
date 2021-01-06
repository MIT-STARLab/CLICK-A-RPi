'''

This is a alias map for fpga map requests.
Please use all human readable names in the code and no individual register values.

All blocks are specified as (starting reg address, length of block)

Human readable value block starts at reg 200;

'''

REGISTERS = [None] * 300 # [encoding, [physical registers, least to most significant bits], rw flag]

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

'''
Human readable
'''

TEMPERATURE_BLOCK_C = [200, 6]

REGISTERS[200] = ['I', [99,98], False] #Temperature 1
REGISTERS[201] = ['I', [101,100], False] #Temperature 2
REGISTERS[202] = ['I', [103,102], False] #Temperature 3
REGISTERS[203] = ['I', [105,104], False] #Temperature 4
REGISTERS[204] = ['I', [107,106], False] #Temperature 5
REGISTERS[205] = ['I', [109,108], False] #Temperature 6

TEMPERATURE_BLOCK_C = [206, 4]
REGISTERS[206] = ['I', [111,110], False] #Current consumption LD Bias
REGISTERS[207] = ['I', [113,112], False] #Current consumption TEC
REGISTERS[208] = ['I', [115,114], False] #Current consumption HEATER
REGISTERS[209] = ['I', [117,116], False] #Current consumption EDFA
