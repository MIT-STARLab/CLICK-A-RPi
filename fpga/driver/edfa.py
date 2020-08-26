# Joe Kusters
# 5/Time/Is/Meaningless/20
# edfa.py
# edfa interfacing functions

import fl

def edfa_write_cmd(handle, cmd):
	i = 0
	while i < len(cmd):
		status = fl.flReadChannel(handle, 63)
		status = status & 0x40
		if status==0x40:
			fl.flWriteChannel(handle, 11, cmd[i])
			i+=1 
	return

def edfa_read(handle):
	status = fl.flReadChannel(handle, 63)
	status = status & 0x20
	if status==0x20:
		return fl.flReadChannel(handle, 65)
	else:	
		return None
