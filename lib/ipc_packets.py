import struct

class IpcPacket:
    def __init__(self): pass
    
class TxPacket:
    def __init__(self): IpcPacket.__init__(self)
    
    def encode(self, APID=0x01, payload=''):
        '''Encode a packet to be transmited to the bus:
        APID: BCT APID presneted to the bus
        payload: raw contents, bytes
        returns message bytes'''
        
        self.APID = APID
        self.size = len(payload)
        self.payload = payload
        
        if self.payload:
            self.raw = struct.pack('BH%ds'%self.size,APID,self.size,payload)
        else:
            self.raw = struct.pack('BH',APID,self.size)
        
        return self.raw
        
    def decode(self, raw):
        '''Decode a packet to be transmited to the bus:
        raw: message bytes to decode 
        returns APID: BCT APID presneted to the bus
        and payload: raw contents, bytes'''
        
        self.raw = raw
        raw_size = len(raw)-4
        
        self.APID, self.size, self.payload = struct.unpack('BH%ds'%raw_size,raw)
        
        assert self.size == raw_size
        
        return self.APID, self.payload
        
    def __str__(self):
        if self.payload:
            return 'IPC TX_PACKETS, APID:0x%02X, size:%d, payload:0x%X' % (self.APID, self.size, int.from_bytes(self.payload,'big'))
        else: 
            return 'IPC TX_PACKETS, APID:0x%02X, size:%d' % (self.APID, self.size)
 
if __name__ == '__main__':

    empty_ipc_txpacket = TxPacket()
    raw = empty_ipc_txpacket.encode(APID=0x02)
    empty_ipc_txpacket.decode(raw)
    print(empty_ipc_txpacket)
    
    not_empty_ipc_txpacket = TxPacket()
    raw = not_empty_ipc_txpacket.encode(APID=0x02,payload=b"Je s'appelle Groot")
    not_empty_ipc_txpacket.decode(raw)
    print(not_empty_ipc_txpacket)