import struct

class IpcPacket:
    def __init__(self):
        self.raw = ''
    
class TxPacket:
    def __init__(self):
    
    def encode(self,APID=0x01,payload ='')
        '''Encode a packet to be transmited to the bus:
        APID: BCT APID presneted to the bus
        payload: raw contents, bytes
        returns message bytes'''
        
        self.APID = APID
        self.size = len(payload)
        self.payload = payload
        
        self.raw = struct.pack('cH%ds'%self.size,APID,self.size,payload)
        
        return self.raw
        
    def decode(raw):
        '''Decode a packet to be transmited to the bus:
        raw: message bytes to decode 
        returns APID: BCT APID presneted to the bus
        and payload: raw contents, bytes'''
        
        self.raw = raw
        raw_size = len(raw)-4
        
        self.APID, self.size, self.payload = struct.unpack('cH%ds'%raw_size)
        
        assert self.size = raw_size
        
        return self.APID, self.payload
        
    def __str__(self):
        if self.payload:
            return 'IPC TX_PACKETS, APID:%02X, size:%d, payload:'
        else: 
    
    