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
        
class FPGAMapRequestPacket(IpcPacket):
    def __init__(self): IpcPacket.__init__(self)
    
    def encode(self, return_addr, rq_number, rw_flag, start_addr, read_size=0, write_data=''):
        '''Encode a request to the FPGA memory map
        return_addr: unique identifier for return message, can be PID
        request_number: will be sent back with the answer, can be used to identify individual requests
        rw_flag: read=0, write=1
        start_addr: first FPGA register to read or write
        read_size: number of 32bit registers to read OR write_data: data to be written
        returns
        message bytes'''
        
        self.return_addr = return_addr
        self.rq_number = rq_number
        self.rw_flag = rw_flag
        self.start_addr = start_addr
        self.read_size = read_size
        self.write_data = write_data
        self.write_size = len(write_data)
        
        assert (read_size > 0) ^ (self.write_size > 0)
        
        if read_size > 0:
            #Reading
            self.raw = struct.pack('IBBHI',return_addr,rq_number,rw_flag,start_addr,read_size)
            assert self.rw_flag == 0
        else:
            #Writing
            assert (self.write_size % 4) == 0
            assert self.rw_flag == 1
            self.raw = struct.pack('IBBH%ds'%self.write_size,return_addr,rq_number,rw_flag,start_addr,write_data)
            
        return self.raw
    
    def decode(self, raw):
        '''Decode a request to the FPGA memory map
        raw: message bytes to decode
        returns
        return_addr: unique identifier for return message, can be PID
        request_number: will be sent back with the answer, can be used to identify individual requests
        rw_flag: read=0, write=1
        start_addr: first FPGA register to read or write
        read_size: number of 32bit registers to read OR write_data: data to be written'''
        
        self.raw = raw
        raw_size = len(raw)-8
        
        self.return_addr, self.rq_number, self.rw_flag, self.start_addr, self.read_or_write_content = struct.unpack('IBBH%ds'%raw_size,raw)
        
        if self.rw_flag == 0:
            #Reading
            assert raw_size == 4
            self.read_size = struct.unpack('I',self.read_or_write_content)[0]
            
            return self.return_addr, self.rq_number, self.rw_flag, self.start_addr, self.read_size
            
        elif self.rw_flag == 1:
            #Writing
            assert (raw_size % 4) == 0
            self.write_data = self.read_or_write_content
            
            return self.return_addr, self.rq_number, self.rw_flag, self.start_addr, self.write_data
            
        else:
            raise
            
    def __str__(self):
        if self.rw_flag == 0:
            return 'IPC FPGA_MAP_REQUEST_PACKET, PID:%d, request number:%d, Reading from address:0x%04X, size:%d' % (self.return_addr, self.rq_number, self.start_addr, self.read_size)
        elif self.rw_flag == 1: 
            return 'IPC FPGA_MAP_REQUEST_PACKET, PID:%d, request number:%d, Writing to address:0x%04X, data:0x%X' % (self.return_addr, self.rq_number, self.start_addr, int.from_bytes(self.write_data,'big'))

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
    raw = not_empty_ipc_rxpatpacket.encode(APID=0x06,ts_txed_s=567,ts_txed_ms=4,payload=b"Je s'appelle Groot...")
    not_empty_ipc_rxpatpacket.decode(raw)
    print(not_empty_ipc_rxpatpacket)
    
    ips_heartbeatpacket = HandlerHeartbeatPacket()
    raw = ips_heartbeatpacket.encode(origin=123,ts_txed_s=456)
    ips_heartbeatpacket.decode(raw)
    print(ips_heartbeatpacket)
    
    ipc_fpgarqpacket_read = FPGAMapRequestPacket()
    raw = ipc_fpgarqpacket_read.encode(return_addr=456, rq_number=123, rw_flag=0, start_addr=0x1234, read_size=456)
    ipc_fpgarqpacket_read.decode(raw)
    print(ipc_fpgarqpacket_read)
    
    ipc_fpgarqpacket_write = FPGAMapRequestPacket()
    raw = ipc_fpgarqpacket_write.encode(return_addr=456, rq_number=123, rw_flag=1, start_addr=0xABCD, write_data=b"Je s'appelle Groot!!")
    ipc_fpgarqpacket_write.decode(raw)
    print(ipc_fpgarqpacket_write)