import fsm_spi_driver as fsm
import argparse

parser = argparse.ArgumentParser(description='Send FSM Commands over RPi SPI for testing')
parser.add_argument('--init', action='store_true', help='send the 4 commands to initialize the FSM')
parser.add_argument('--write', action='store', type=int, nargs=2, help='write the given voltage to FSM DAC given address')
parser.add_argument('--update', action='store', type=int, nargs=1, help='update the given DAC address')
parser.add_argument('--send', action='store', type=int, nargs=3, help='send the given command, address, and value to the DAC')

args = parser.parse_args()

if args.init:
    print('Initializing FSM DAC...')
    fsm.init()
    print('done!')

if args.write:
    print('Writing '+str(args.write[1])+' to FSM Out '+str(args.write[0]))
    fsm.write_dac(args.write[0], args.write[1])
    print('done!')

if args.update:
    print('Updating DAC Output '+str(args.update[0]))
    fsm.update_dac(args.update[0])
    print('done!')

if args.send:
    print('Sending command '+str(args.send[0])+' to Address '+str(args.send[1])+' with value '+str(args.send[2]))
    fsm.send_command(args.send[0], args.send[1], args.send[2])
    print('done!')

