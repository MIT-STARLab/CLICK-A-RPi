# Joe Kusters
# random_data.py
# makes a random binary data file for testing the modulator

import random as rand

f = open('/home/pi/NODE-FSW/fpga/poem_zone.txt', 'wb')
for i in range(0, 10000):
    f.write(str(i)+'\n')
f.close()
