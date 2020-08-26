#include <wiringPiSPI.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

char check_sync(char *buffer){
	char sync[4] = {0x35, 0x2E, 0xF8, 0x53};
	int i = 0;
	char sync_good = 1;
	for(i = 0; i<4; i++){
		if(sync[i] != buffer[i]){sync_good = 0;}
	}
	return sync_good;
}

int main(){
	
	int spi0 = wiringPiSPISetupMode(0, 12000000, 0);
	
	int i;
	int j;
	int len;
	int errors = 0;
	char cycle = 0;
	char num = 0;
	FILE *f = fopen("dummy_packets.bin", "rb");
	char **packets = (char **) malloc(256*sizeof(char *));
	for(i=0; i<256; i++){
		packets[i] = (char *) malloc(512*sizeof(char));
		fread(packets[i], sizeof(char), 10, f);
		len = (packets[i][8]<<8)+packets[i][9];
		fread(&packets[i][10], sizeof(char), len+1, f);
	}
	char *buffer = (char *) malloc(512*sizeof(char));
	while(1){
		memset(buffer, 0, 512);
		wiringPiSPIDataRW(0, buffer, 10);
		if(check_sync(buffer)){
			int len_read = (buffer[8]<<8)+buffer[9];
			wiringPiSPIDataRW(0, &buffer[10], len_read+1);
            if(((buffer[4]&0x2)==0x2) & (buffer[5]==0x50)){
                usleep(100000); 
                wiringPiSPIDataRW(0, buffer, len_read+11);
			} else if(buffer[10]==0x11){cycle = 1;} 
			else if(buffer[10]==0x22){cycle = 0;} 
			else if(buffer[10]==0x33){
				char ind = buffer[11];
				len = (packets[ind][8]<<8)+packets[ind][9];
				memcpy(buffer, packets[ind], 11+len); 
				usleep(100000);
				wiringPiSPIDataRW(0, buffer, 11+len); 
				
			} 
		}
		if(cycle){
			len = (packets[num][8]<<8)+packets[num][9];
			memcpy(buffer, packets[num], 11+len);
			usleep(100000);
			wiringPiSPIDataRW(0, buffer, 11+len);
			num++;
			if(num==256){num=0;}
		}
		usleep(100000);
	}
	close(spi0);
	for(i=0; i<256; i++){free(packets[i]);}
	free(packets);
	free(buffer);
	fclose(f);
	return 0;
}
