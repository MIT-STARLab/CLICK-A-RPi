import zmq
import ipc_packets
import options
import os
import struct
import fpga_map
import collections

class FPGAClientInterface:
    def __init__(self,context=None):
        
        # use PID as unique identifier for this progress
        self.pid = os.getpid()
        
        # ZeroMQ context setup
        if not context: context = zmq.Context()
        
        # FPGA request socket
        self.socket_request = context.socket(zmq.PUB)
        self.socket_request.connect("tcp://localhost:%s" % options.FPGA_MAP_REQUEST_PORT)
        
        # FPGA answer socket
        self.socket_answer = context.socket(zmq.SUB)
        self.socket_answer.setsockopt(zmq.SUBSCRIBE, struct.pack('I',self.pid)) # Filter only your own packets
        self.socket_answer.connect("tcp://localhost:%s" % options.FPGA_MAP_ANSWER_PORT)
        
        self._token = 0
        self._pending = collections.deque()
        
        # checking if FPGA is here for 5 sec.
        tries = 50
        self.socket_answer.setsockopt(zmq.RCVTIMEO, 100) # Set timout
        while 1:
            try:
                self.write_reg(0,'')
                break
            except zmq.ZMQError as error:
                tries -= 1
                if tries == 0:
                    raise error   
        self.socket_answer.setsockopt(zmq.RCVTIMEO, options.MESSAGE_TIMEOUT) # Set timout
        
        self.edfa = fpga_map.EDFA(self)
        
    class _Future:
        def __init__(self,handler,token,addr,wr_flag,size):
            self.handler = handler
            self.token = token
            self.addr = addr
            self.wr_flag = wr_flag
            self.size = size
            self.value = None
            self.valid = False
            self.time_out = False
            
        def wait(self):
            if not self.valid: self.handler._receive(self)
            return self.value
                
    def verify_boot(self):
        try: reg0 = self.read_reg(0)
        except ValueError: return False
        dcm_locked = reg0 & 1
        crc_err = (reg0 >> 4) & 1
        return (dcm_locked and not crc_err)
    
    def _get_token(self):
        self._token = (self._token+1) % 255
        return self._token
    
    def read_reg_async(self,addr,size=1):
        '''Write registers and send typed result'''
        
        request_id = self._get_token()
    
        # send request
        self._send_request(
            request_number = request_id,
            start_address = addr,
            rw_flag = 0,
            size = size*4,
            data = '')
    
        future = FPGAClientInterface._Future(self,request_id,addr,0,4*size)
        self._pending.append(future)
        return future
    
    def write_reg_async(self,addr,value):
        '''Write registers:
        - addr: start address
        - value:
            int will be sent as unsigned 32 bits int
            list of ints will be sent as len * unsigned 32 bits ints
            str or bytes will be sent directly if len(str) is a multiple of 4'''
            
        request_id = self._get_token()
            
        # encode payload based on type
        if type(value) is int:
            len_pck = 4
            payload = struct.pack('I',value)
        elif type(value) is list:
            len_pck = len(value)
            payload = struct.pack('%dI'%len_pck,*value)
            len_pck *= 4
        elif type(value) is str:
            lstr = len(value)
            value = value + 'X'*(-lstr%4)
            payload = struct.pack('I',lstr) + value
            len_pck = len(payload)
        else:
            raise TypeError('Can only send int, list, or string')
        
        # send request
        self._send_request(
            request_number = request_id,
            start_address = addr,
            rw_flag = 1,
            size = len_pck,
            data = payload)
           
        future = FPGAClientInterface._Future(self,request_id,addr,1,len_pck)
        self._pending.append(future)
        return future
        
    def write_reg(self,addr,value):
        future = self.write_reg_async(addr,value)
        return future.wait()
    
    def read_reg(self,addr,size=1):
        future = self.read_reg_async(addr,size)
        return future.wait()
        
    def _send_request(self,request_number,start_address,rw_flag,size,data):
        '''format and send the request IPC packets'''
        pck = ipc_packets.FPGAMapRequestPacket()
        raw = pck.encode(
            return_addr=self.pid,
            rq_number=request_number,
            rw_flag=rw_flag,
            start_addr=start_address,
            size=size,
            write_data=data)
        self.socket_request.send(raw)
        
    def _get_packet(self):
    
        try:
            message = self.socket_answer.recv()
        except (zmq.ZMQError, zmq.NotDone, zmq.ContextTerminated) as error:
            for future in self._pending:
                future.time_out = True
            self._pending = collections.deque()
            raise error
            
        pck = ipc_packets.FPGAMapAnswerPacket()
        pck.decode(message)
        return pck

    def _receive(self,target_future):
    
        current_fut = None
    
        while current_fut != target_future:
            
            current_fut = self._pending.popleft()
            
            pck = self._get_packet()
            
            if options.CHECK_ASSERTS:
                assert pck.start_addr == current_fut.addr
                assert pck.rq_number == current_fut.token
                assert pck.rw_flag == current_fut.wr_flag
                #assert pck.size == current_fut.size
            
            if pck.error:
                raise ValueError('Error flag set on FPGA answer')
                
            current_fut.value = self._decode_payload(pck)
            current_fut.valid = True
    
    def _decode_payload(self,pck):
        if not pck.size: return None
        if fpga_map.REGISTER_TYPE[pck.start_addr] == 'str':
            lstr = struct.unpack('I',pck.read_data[0:4])[0]
            return pck.read_data[4:lstr+4]
        elif pck.size > 4:
            reg_size = pck.size/4
            addr_range = range(pck.start_addr, pck.start_addr+4)
            raw_slices = [pck.read_data[0+i:4+i] for i in range(0, pck.size, 4)]
            return [self._decode_reg(addr,raw) for addr,raw in zip(addr_range,raw_slices)]
        else:
            return self._decode_reg(pck.start_addr,pck.read_data) 
            
    def _decode_reg(self,addr,raw):
        return struct.unpack(fpga_map.REGISTER_TYPE[addr],raw)[0]
        
    def wait_all(self,*arg):
        return [a.wait() for a in arg] 
        
class FPGAServerInterface:
    def __init__(self,context=None,timeout=-1):
        
        # ZeroMQ context setup
        if not context: context = zmq.Context()
        
        self.socket_request = context.socket(zmq.SUB)
        self.socket_request.bind("tcp://*:%s" % options.FPGA_MAP_REQUEST_PORT)
        if timeout != -1:
            self.socket_request.setsockopt(zmq.RCVTIMEO, timeout) # Set timout 
        self.socket_request.setsockopt(zmq.SUBSCRIBE, b'') # subscribe to ALL incoming FPGA_map_requests

        self.socket_answer = context.socket(zmq.PUB)
        self.socket_answer.bind("tcp://*:%s" % options.FPGA_MAP_ANSWER_PORT)
        
    class Request(ipc_packets.FPGAMapRequestPacket):
        def __init__(self,handler):
            ipc_packets.FPGAMapRequestPacket.__init__(self)
            self.handler = handler
            
        def answer(self,data,error_flag=0):
        
            enc_data = self._encode_payload(self.start_addr,data)
            
            asw_pck = ipc_packets.FPGAMapAnswerPacket()
            raw = asw_pck.encode(
                return_addr = self.return_addr,
                rq_number = self.rq_number,
                rw_flag = self.rw_flag,
                error_flag = error_flag,
                start_addr = self.start_addr,
                size = len(enc_data),
                read_data = enc_data)
                
            self.handler.send_answer(raw)
            
        def _encode_payload(self,addr,value):
            
            if fpga_map.REGISTER_TYPE[addr] == 'str':
                lstr = len(value)
                value = value + 'X'*(-lstr%4)
                return struct.pack('I',lstr) + value
                
            elif type(value) is list:
                raw = [self._encode_reg(a,v) for a,v in zip(range(addr,addr+len(value)),value)]
                return ''.join(raw)
            
            elif value == '': return ''
            
            else : return self._encode_reg(addr,value)
            
        def _encode_reg(self,addr,value):
            return struct.pack(fpga_map.REGISTER_TYPE[addr],value)
        
    def get_request(self):
        
        message = self.socket_request.recv()
        
        pck = FPGAServerInterface.Request(self)
        pck.decode(message)
        
        return pck
        
    def send_answer(self,raw):
        self.socket_answer.send(raw)
       

# =========================== Test and example ===========================

#dummy server for tests       
def loop():
    import time
    print('Subprocess launched')
    server = FPGAServerInterface(timeout=10)
    print('Server started')
    start_time = time.time()
    while 1:
        try:
            req = server.get_request()
            start_time = time.time()
        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                elapsed = time.time() - start_time
                if elapsed > 3: break
                else: continue
            else: raise e
        if not req.rw_flag: req.answer(range(req.size//4))
        else: req.answer('')
    print('Server closed')
        
if __name__ == '__main__':

    import multiprocessing
    server_loop = multiprocessing.Process(target=loop)
    server_loop.start()
    
    fpga = FPGAClientInterface()
    
    print('Reading register 10')
    print(fpga.read_reg(10))
    
    print('Reading register 1, 2, 3')
    print(fpga.read_reg(1,3))
    
    print('Writing register 10')
    fpga.write_reg(10,85)
    
    print('Writing register 4, 5, 6')
    fpga.write_reg(4,[1,2,3])
    
    print('Staging reads and writes')
    cbr1 = fpga.read_reg_async(10)
    cbr2 = fpga.read_reg_async(1,3)
    cbw1 = fpga.write_reg_async(10,85)
    cbw2 = fpga.write_reg_async(4,[1,2,3])
    
    # Do other stuff while waiting...
    
    r2, r1, w1, w2 = fpga.wait_all(cbr2,cbr1,cbw1,cbw2)
    print(r1)
    print(r2)
    
    server_loop.join()
