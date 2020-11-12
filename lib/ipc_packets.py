import struct

class IpcPacket:
    def __init__(self): pass
    
class TxPacket(IpcPacket):
    def __init__(self): IpcPacket.__init__(self)
    
    def encode(self, APID=0x01, payload=''):
        '''Encode a packet to be transmited to the bus:
        APID: BCT APID presneted to the bus
        payload: raw contents, bytes
        returns
        message bytes'''
        
        self.APID = APID
        self.size = len(payload)
        self.payload = payload
        
        if self.payload:
            self.raw = struct.pack('BxH%ds'%self.size,APID,self.size,payload)
        else:
            self.raw = struct.pack('BxH',APID,self.size)
        
        return self.raw
        
    def decode(self, raw):
        '''Decode a packet to be transmited to the bus:
        raw: message bytes to decode 
        returns
        APID: BCT APID presneted to the bus
        payload: raw contents, bytes'''
        
        self.raw = raw
        raw_size = len(raw)-4
        
        self.APID, self.size, self.payload = struct.unpack('BxH%ds'%raw_size,raw)
        
        assert self.size == raw_size
        
        return self.APID, self.payload
        
    def __str__(self):
        if self.payload:
            return 'IPC TX_PACKET, APID:0x%02X, size:%d, payload:0x%X' % (self.APID, self.size, int.from_bytes(self.payload,'big'))
        else: 
            return 'IPC TX_PACKET, APID:0x%02X, size:%d' % (self.APID, self.size)
 
class RxCommandPacket(IpcPacket):
    def __init__(self): IpcPacket.__init__(self)
    
    def encode(self, APID=0x01, ts_txed_s=0, ts_txed_ms=0, payload=''):
        '''Encode a packet that have been received from the bus:
        APID: BCT APID from the bus
        ts_txed_s: Bus timestamp at witch it transmited the packet to the CLICK payload, in seconds
        ts_txed_s: Bus timestamp, in x200 ms
        payload: raw contents, bytes
        returns
        message bytes'''
        
        self.APID = APID
        self.ts_txed_s = ts_txed_s
        self.ts_txed_ms = ts_txed_ms
        self.size = len(payload)
        self.payload = payload
        
        if self.payload:
            self.raw = struct.pack('BBHI%ds'%self.size,APID,ts_txed_ms,self.size,ts_txed_s,payload)
        else:
            self.raw = struct.pack('BBHI',APID,ts_txed_ms,self.size,ts_txed_s)
        
        return self.raw
        
    def decode(self, raw):
        '''Decode a packet that have been received from the bus:
        raw: message bytes to decode 
        returns
        APID: BCT APID presneted to the bus
        ts_txed_s: Bus timestamp at witch it transmited the packet to the CLICK payload, in seconds
        ts_txed_s: Bus timestamp, in x200 ms
        payload: raw contents, bytes'''
        
        self.raw = raw
        raw_size = len(raw)-8
        
        self.APID, self.ts_txed_ms, self.size, self.ts_txed_s, self.payload = struct.unpack('BBHI%ds'%raw_size,raw)
        
        assert self.size == raw_size
        
        return self.APID, self.ts_txed_s, self.ts_txed_ms, self.payload
        
    def __str__(self):
        if self.payload:
            return 'IPC RX_COMMAND_PACKET, APID:0x%02X, time:%d.%d s, size:%d, payload:0x%X' % (self.APID, self.ts_txed_s, 2*self.ts_txed_ms, self.size, int.from_bytes(self.payload,'big'))
        else: 
            return 'IPC RX_COMMAND_PACKET, APID:0x%02X, time:%d.%d s, size:%d' % (self.APID, self.ts_txed_s, 2*self.ts_txed_ms, self.size)

class RxPATPacket(RxCommandPacket):
    def __init__(self): RxCommandPacket.__init__(self)
    
    def __str__(self):
        if self.payload:
            return 'IPC RX_PAT_PACKET, APID:0x%02X, time:%d.%d s, size:%d, payload:0x%X' % (self.APID, self.ts_txed_s, 2*self.ts_txed_ms, self.size, int.from_bytes(self.payload,'big'))
        else: 
            return 'IPC RX_PAT_PACKET, APID:0x%02X, time:%d.%d s, size:%d' % (self.APID, self.ts_txed_s, 2*self.ts_txed_ms, self.size)    

class HandlerHeartbeatPacket(IpcPacket):
    def __init__(self): IpcPacket.__init__(self)
    
    def encode(self, origin=0, ts_txed_s=0):
        '''Encode a heartbeat packet:
        origin: Heartbit sender, can be PID
        ts_txed_s: Timestamp at witch the heartbeat is sent, in seconds
        returns
        message bytes'''
        
        self.origin = origin
        self.ts_txed_s = ts_txed_s
        
        self.raw = struct.pack('II',origin,ts_txed_s)

        return self.raw
        
    def decode(self, raw):
        '''Decode a heartbeat packet:
        raw: message bytes to decode 
        returns
        origin: Heartbit sender, can be PID
        ts_txed_s: Timestamp at witch the heartbeat is sent, in seconds'''
        
        self.raw = raw
        
        self.origin, self.ts_txed_s, = struct.unpack('II',raw)
        
        return self.origin, self.ts_txed_s
        
    def __str__(self):
        return 'IPC HANDLER_HEARTBEAT_PACKET, PID:%d, time:%d s' % (self.origin, self.ts_txed_s)

if __name__ == '__main__':

    empty_ipc_txpacket = TxPacket()
    raw = empty_ipc_txpacket.encode(APID=0x01)
    empty_ipc_txpacket.decode(raw)
    print(empty_ipc_txpacket)
    
    not_empty_ipc_txpacket = TxPacket()
    raw = not_empty_ipc_txpacket.encode(APID=0x02,payload=b"Je s'appelle Groot")
    not_empty_ipc_txpacket.decode(raw)
    print(not_empty_ipc_txpacket)
    
    empty_ipc_rxcompacket = RxCommandPacket()
    raw = empty_ipc_rxcompacket.encode(APID=0x03,ts_txed_s=123,ts_txed_ms=2)
    empty_ipc_rxcompacket.decode(raw)
    print(empty_ipc_rxcompacket)
    
    not_empty_ipc_rxcompacket = RxCommandPacket()
    raw = not_empty_ipc_rxcompacket.encode(APID=0x04,ts_txed_s=567,ts_txed_ms=4,payload=b"Je s'appelle Groot?")
    not_empty_ipc_rxcompacket.decode(raw)
    print(not_empty_ipc_rxcompacket)
    
    empty_ipc_rxpatpacket = RxPATPacket()
    raw = empty_ipc_rxpatpacket.encode(APID=0x05,ts_txed_s=123,ts_txed_ms=2)
    empty_ipc_rxpatpacket.decode(raw)
    print(empty_ipc_rxpatpacket)
    
    not_empty_ipc_rxpatpacket = RxPATPacket()
    raw = not_empty_ipc_rxpatpacket.encode(APID=0x06,ts_txed_s=567,ts_txed_ms=4,payload=b"Je s'appelle Groot!!")
    not_empty_ipc_rxpatpacket.decode(raw)
    print(not_empty_ipc_rxpatpacket)
    
    ips_heartbeatpacket = HandlerHeartbeatPacket()
    raw = ips_heartbeatpacket.encode(origin=123,ts_txed_s=456)
    ips_heartbeatpacket.decode(raw)
    print(ips_heartbeatpacket)