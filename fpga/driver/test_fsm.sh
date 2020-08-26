#!/bin/bash
derp=1
i=0
while [ $derp -lt 2 ] 
do
	if [ $i -gt 255 ]
	then
		i=0
	fi

	python fpgadriver.py -i  04b4:8613 -v 1d50:602b:0002 --write 8 23
	sleep 0.001
	python fpgadriver.py -i  04b4:8613 -v 1d50:602b:0002 --write 9 0
	sleep 0.001
	python fpgadriver.py -i  04b4:8613 -v 1d50:602b:0002 --write 10 $i
	i=$[$i+1]
	echo $i
	sleep 1
done
