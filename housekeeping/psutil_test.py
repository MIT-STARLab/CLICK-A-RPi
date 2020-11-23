import psutil


def psutil_test():

    camera_pid = 0
    camera_cpu = 0
    camera_mem = 0
    camera_vmem = 0

    for p in psutil.process_iter(['pid','name','cpu_percent','memory_percent']):
        if p.info['name'] == 'pppd':
            print('found!')
            camera_pid = p.pid
            camera_cpu = p.cpu_percent()
            camera_mem = p.memory_percent('rss')
            camera_vmem = p.memory_percent('vms')
        elif p.info['name'] == 'commandhandler':
            ch_pid = p.pid
            ch_cpu = p.cpu_percent()
            ch_mem = p.memory_percent('rss')
            ch_vmem = p.memory_percent('vms')
        elif p.info['name'] == 'fpga':
            fpga_pid = p.pid
            fpga_cpu = p.cpu_percent()
            fpga_mem = p.memory_percent('rss')
            fpga_vmem = p.memory_percent('vms')

    print(camera_pid)
    print(camera_cpu)
    print(camera_mem)
    print(camera_vmem)

if __name__ == '__main__':
    psutil_test()
