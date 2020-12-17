

sudo python fpgadriver.py -i 1d50:602b -v 1d50:602b:0002 --write 32 85
wait .01
sudo python fpgadriver.py -i 1d50:602b -v 1d50:602b:0002 --write 32 15
wait .01
sudo python fpgadriver.py -i 1d50:602b -v 1d50:602b:0002 --write 32 85
