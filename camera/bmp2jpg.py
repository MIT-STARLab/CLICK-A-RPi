import os
import sys

#convert shell command requires imagemagick:
#sudo apt-get install imagemagick --fix-missing

fileName_bmp = sys.argv[1] #cmd line call is python bmp2jpg.py <fileName.bmp>
cmdName = 'convert ' + fileName_bmp + ' ' + fileName[0:(len(fileName)-3)] + 'jpg'
#print(cmdName)
os.system(cmdName)
