#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "fl.h"

#define BUFSIZE 512
#define DELAY_US 25
#define TIMEOUT_US 50000

int main (int argc, const char *argv[])
{
    struct FLContext *handle = NULL;
    char *cmd = "fline\r", reply[BUFSIZE];
    uint8 len = strlen(cmd), flag = 0, i = 0, success = 0;
    uint16 tx_delay = 0, inter_delay = 0, rx_delay = 0;
    uint32 timeout = TIMEOUT_US;

    if(fpgalink_init(&handle) == FL_SUCCESS)
    {
        /* Write fline command to EDFA */
        while (i < len && timeout > 0)
        {
            if (fpgalink_read(handle, 63, &flag) == FL_SUCCESS)
            {
                if ((flag & 0x40) == 0x40)
                {
                    if (fpgalink_write(handle, 11, cmd[i]) == FL_SUCCESS)
                    {
                        i++;
                    }
                    else break;
                }
                else
                {
                    usleep(DELAY_US);
                    timeout -= DELAY_US;
                }
            }
            else break;
        }

        if (timeout < DELAY_US)
        {
            fprintf(stderr, "EDFA write timeout\n");
        }
        else if (i != len)
        {
            fprintf(stderr, "FPGALink error\n");
        }

        timeout = TIMEOUT_US;

        /* Read EDFA reply */
        if (i == len)
        {
            len = 0;
            while(timeout > 0)
            {
                if (fpgalink_read(handle, 63, &flag) == FL_SUCCESS)
                {
                    if ((flag & 0x20) == 0x20)
                    {
                        if (fpgalink_read(handle, 65, (uint8*) (reply + len)) == FL_SUCCESS)
                        {
                            len++;
                            if (len > BUFSIZE)
                            {
                                fprintf(stderr, "\nRead buffer overflow\n");
                                break;
                            }
                            else if (reply[len-2] == '\r' && reply[len-1] == '\n')
                            {
                                if (len > 15)
                                {
                                    reply[len-2] = '\0';
                                    printf("%s\n", reply);
                                    break;
                                }
                                len = 0;
                            }
                        }
                        else break;
                    }
                    else
                    {
                        usleep(DELAY_US);
                        timeout -= DELAY_US;
                    }
                }
                else break;
            }
        }
        if (timeout < DELAY_US)
        {
            fprintf(stderr, "EDFA write timeout\n");
        }

        flClose(handle);
    }
    else fprintf(stderr, "FPGALink init failed\n");
    return 0;
}
