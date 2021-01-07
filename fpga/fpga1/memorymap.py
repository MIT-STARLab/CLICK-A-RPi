# Joe Kusters
# memorymap.py
# implements the FPGA memory map has an object which takes in a string and returns the register number for that string

class MemoryMap:
    def __init__(self):
        # initialize memory map with values from memory_map.xlsm, all values in decimal
        # control & status registers
        self.ctr = 0
        self.flg = 63
        self.acc = 49
        self.rcc = 50
        self.lac = 51
        self.lrc = 52
        self.frc = 53
        self.ver = 54
        self.wrs = 55
        self.wra = 56
        # calibration laser register
        self.cal = 32
        # power management registers
        self.po1 = 33
        self.po2 = 34
        self.po3 = 35
        self.po4 = 36
        self.cc1a = 96
        self.cc1b = 97
        self.cc2a = 98
        self.cc2b = 99
        self.cc3a = 100
        self.cc3b = 101
        self.cc4a = 102
        self.cc4b = 103
        # thermal management registers
        self.te1a = 104
        self.te1b = 105
        self.te2a = 106
        self.te2b = 107
        self.te3a = 108
        self.te3b = 109
        self.te4a = 110
        self.te4b = 111
        self.te5a = 112
        self.te5b = 113
        self.te6a = 114
        self.te6b = 115
        self.he1 = 37
        self.he2 = 38
        # LD temp controller registers
        self.LTSa = 1
        self.LTSb = 2
        self.LTMa = 116
        self.LTMb = 117
        # LD Bias Current controller registers
        self.LBCa = 3
        self.LBCb = 4
        # Tx power level registers
        self.THRa = 5
        self.THRb = 6
        self.THRc = 7
        self.pdi = 64
        # FSM registers
        self.FSMa = 8
        self.FSMb = 9
        self.FSMc = 10
        # EDFA registers
        self.etx = 11
        self.erx = 65
        # SEM controller registers
        self.sep = 12
        self.sce = 66
        self.sie = 67
        self.sst = 68
        # Modulator registers
        self.dat = 13
        self.delay = 14
        self.fff = 57
        self.err = 58
        self.one = 59
        self.dfp = 60
        self.efp = 61
        self.ofp = 62

    def get_register(self, name):
        if(name == 'CTR' or name == 'ctr' or name == 'Control' or name == 'control'):
            return self.ctr
        elif(name == 'FLG' or name == 'flg' or name == 'Status' or name == 'status'):
            return self.flg
        elif(name == 'ACC' or name == 'acc' or name == 'Accepted Commands' or name == 'accepted commands'):
            return self.acc
        elif(name == 'RCC' or name == 'rcc' or name == 'Rejected Commands' or name == 'rejected commands'):
            return self.rcc
        elif(name == 'LAC' or name == 'lac' or name == 'Last Accepted Command' or name == 'last accepted command'):
            return self.lac
        elif(name == 'LRC' or name == 'lrc' or name == 'Last Rejected Command' or name == 'last rejected command'):
            return self.lrc
        elif(name == 'FRC' or name == 'frc' or name == 'Free Running Counter' or name == 'free running counter'):
            return self.frc
        elif(name == 'VER' or name == 'ver' or name == 'Core Version' or name == 'core version'):
            return self.ver
        elif(name == 'WRS' or name == 'wrs' or name == 'Write Accesses' or name == 'write accesses'):
            return self.wrs
        elif(name == 'WRA' or name == 'wra' or name == 'Last Write Address' or name == 'last write address'):
            return self.wra
        elif(name == 'CAL' or name == 'cal' or name == 'Calibration Diode' or name == 'calibration diode'):
            return self.cal
        elif(name == 'PO1' or name == 'po1' or name == 'Power On/Off 1' or name == 'power on/off 1' or name == 'Power on/off 1'):
            return self.po1
        elif(name == 'PO2' or name == 'po2' or name == 'Power On/Off 2' or name == 'power on/off 2' or name == 'Power on/off 2'):
            return self.po2
        elif(name == 'PO3' or name == 'po3' or name == 'Power On/Off 3' or name == 'power on/off 3' or name == 'Power on/off 3'):
            return self.po3
        elif(name == 'PO4' or name == 'po4' or name == 'LD TEC Power on/of' or name == 'LD TEC power on/off'):
            return self.po4
        elif(name == 'CC1a' or name == 'cc1a' or name == 'Current Consumption 1 (MSB)' or name == 'current consumption 1 (msb)'):
            return self.cc1a
        elif(name == 'CC1b' or name == 'cc1b' or name == 'Current Consumption 1 (LSB)' or name == 'current consumption 1 (lsb)'):
            return self.cc1b
        elif(name == 'CC2a' or name == 'cc2a' or name == 'Current Consumption 2 (MSB)' or name == 'current consumption 2 (msb)'):
            return self.cc2a
        elif(name == 'CC2b' or name == 'cc2b' or name == 'Current Consumption 2 (LSB)' or name == 'current consumption 2 (lsb)'):
            return self.cc2b
        elif(name == 'CC3a' or name == 'cc3a' or name == 'Current Consumption 3 (MSB)' or name == 'current consumption 3 (msb)'):
            return self.cc3a
        elif(name == 'CC3b' or name == 'cc3b' or name == 'Current Consumption 3 (LSB)' or name == 'current consumption 3 (lsb)'):
            return self.cc3b
        elif(name == 'CC4a' or name == 'cc4a' or name == 'Current Consumption 4 (MSB)' or name == 'current consumption 4 (msb)'):
            return self.cc4a
        elif(name == 'CC4b' or name == 'cc4b' or name == 'Current Consumption 4 (LSB)' or name == 'current consumption 4 (lsb)'):
            return self.cc4b
        elif(name == 'TE1a' or name == 'te1a' or name == 'Temperature 1 (MSB)' or name == 'temperature 1 (msb)'):
            return self.te1a
        elif(name == 'TE1b' or name == 'te1b' or name == 'Temperature 1 (LSB)' or name == 'temperature 1 (lsb)'):
            return self.te1b
        elif(name == 'TE2a' or name == 'te2a' or name == 'Temperature 2 (MSB)' or name == 'temperature 2 (msb)'):
            return self.te2a
        elif(name == 'TE2b' or name == 'te2b' or name == 'Temperature 2 (LSB)' or name == 'temperature 2 (lsb)'):
            return self.te2b
        elif(name == 'TE3a' or name == 'te3a' or name == 'Temperature 3 (MSB)' or name == 'temperature 3 (msb)'):
            return self.te3a
        elif(name == 'TE3b' or name == 'te3b' or name == 'Temperature 3 (LSB)' or name == 'temperature 3 (lsb)'):
            return self.te3b
        elif(name == 'TE4a' or name == 'te4a' or name == 'Temperature 4 (MSB)' or name == 'temperature 4 (msb)'):
            return self.te4a
        elif(name == 'TE4b' or name == 'te4b' or name == 'Temperature 4 (LSB)' or name == 'temperature 4 (lsb)'):
            return self.te4b
        elif(name == 'TE5a' or name == 'te5a' or name == 'Temperature 5 (MSB)' or name == 'temperature 5 (msb)'):
            return self.te5a
        elif(name == 'TE5b' or name == 'te5b' or name == 'Temperature 5 (LSB)' or name == 'temperature 5 (lsb)'):
            return self.te5b
        elif(name == 'TE6a' or name == 'te6a' or name == 'Temperature 6 (MSB)' or name == 'temperature 6 (msb)'):
            return self.te6a
        elif(name == 'TE6b' or name == 'te6b' or name == 'Temperature 6 (LSB)' or name == 'temperature 6 (lsb)'):
            return self.te6b
        elif(name == 'HE1' or name == 'he1' or name == 'Heater 1 on/off' or name == 'heater 1 on/off'):
            return self.he1
        elif(name == 'HE2' or name == 'he2' or name == 'Heater 2 on/off' or name == 'heater 2 on/off'):
            return self.he2
        elif(name == 'LTSa' or name == 'ltsa' or name == 'Temp Set Point (MSB)' or name == 'temp set point (msb)'):
            return self.LTSa
        elif(name == 'LTSb' or name == 'ltsb' or name == 'Temp Set Point (LSB)' or name == 'temp set point (lsb)'):
            return self.LTSb
        elif(name == 'LTMa' or name == 'ltma' or name == 'Measured Temp (MSB)' or name == 'measured temp (msb)'):
            return self.LTMa
        elif(name == 'LTMb' or name == 'ltmb' or name == 'Measured Temp (LSB)' or name == 'measured temp (lsb)'):
            return self.LTMb
        elif(name == 'LBCa' or name == 'lbca' or name == 'Bis Current (MSB)' or name == 'bias current (msb)'):
            return self.LBCa
        elif(name == 'LBCb' or name == 'lbcb' or name == 'Bias Current (LSB)' or name == 'bias current (lsb)'):
            return self.LBCb
        elif(name == 'THRa' or name == 'thra' or name == 'Threshold Configuration A' or name == 'threshold configruation a'):
            return self.THRa
        elif(name == 'THRb' or name == 'thrb' or name == 'Threshold Configuration B' or name == 'threshold configruation b'):
            return self.THRb
        elif(name == 'THRc' or name == 'thrc' or name == 'Threshold Configuration C' or name == 'threshold configruation c'):
            return self.THRc
        elif(name == 'ETX' or name == 'etx' or name == 'EDFA TX' or name == 'edfa tx'):
            return self.etx
        elif(name == 'ERX' or name == 'erx' or name == 'EDFA RX' or name == 'edfa rx'):
            return self.erx
        elif(name == 'SEP' or name == 'sep' or name == 'Error Period' or name == 'error period'):
            return self.sep
        elif(name == 'SCE' or name == 'sce' or name == 'Corrected Errors' or name == 'corrected errors'):
            return self.sce
        elif(name == 'SIE' or name == 'sie' or name == 'Inserted Errors' or name == 'inserted errors'):
            return self.sie
        elif(name == 'SST' or name == 'sst' or name == 'Status' or name == 'status'):
            return self.sst
        elif(name == 'DAT' or name == 'dat' or name == 'PPM Order/Data' or name == 'ppm order/data'):
            return self.dat
        elif(name == 'DEL' or name == 'del' or name == 'Delay' or name == 'delay'):
            return self.delay
        elif(name == 'FFF' or name == 'fff' or name == 'FIFO Flags' or name == 'fifo flags'):
            return self.fff
        elif(name == 'ERR' or name == 'err' or name == 'Modulator Errors' or name == 'modulator errors'):
            return self.err
        elif(name == 'ONE' or name == 'one' or name == 'Modulator Ones' or name == 'modulator ones'):
            return self.one
        elif(name == 'DFP' or name == 'dfp' or name == 'Data FIFO Pointer' or name == 'data fifo pointer'):
            return self.dfp
        elif(name == 'EFP' or name == 'efp' or name == 'Errors FIFO Pointer' or name == 'errors fifo pointer'):
            return self.efp
        elif(name == 'OFP' or name == 'ofp' or name == 'Ones FIFO Pointer' or name == 'ones fifo pointer'):
            return self.ofp
        else:
            return 'Not in the memory map, please send a different string'
