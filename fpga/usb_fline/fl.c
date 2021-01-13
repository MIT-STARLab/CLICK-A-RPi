#include <stdio.h>
#include <stdlib.h>
#include "fl.h"

/* Initialize FPGALink */
uint8 fpgalink_init(struct FLContext **handle)
{
    const char *vp = "1d50:602b:0002";
    const char *ivp = "04b4:8613";
    const char *error = NULL;

    uint8 flag;
    int retVal = 0; 
    bool isNeroCapable, isCommCapable;
    FLStatus status;
    
    status = flInitialise(0, &error);
    CHECK_STATUS(status, 1, cleanup);
    status = flOpen(vp, handle, NULL);
    
    if (status)
    {
        int count = 60;
        printf("Loading firmware into %s...\n", ivp);
        status = flLoadStandardFirmware(ivp, vp, &error);
        CHECK_STATUS(status, 2, cleanup);
        
        printf("Awaiting renumeration");
        flSleep(1000);

        do {
            printf(".");
            fflush(stdout);
            status = flIsDeviceAvailable(vp, &flag, &error);
            CHECK_STATUS(status, 3, cleanup);
            flSleep(250);
            count--;
        } while (!flag && count);
        printf("\n");

        if (!flag)
        {
            fprintf(stderr, "FPGALink device did not renumerate properly as %s\n", vp);
            FAIL_RET(4, cleanup);
        }
        
        printf("Attempting to open connection to FPGLink device %s again...\n", vp);
        status = flOpen(vp, handle, &error);
        CHECK_STATUS(status, 5, cleanup);
    }

    isNeroCapable = flIsNeroCapable(*handle);
    isCommCapable = flIsCommCapable(*handle, 0x01);

    if (isCommCapable)
    {
        status = flSelectConduit(*handle, 0x01, &error);
        CHECK_STATUS(status, 21, cleanup);
        return 0;
    }
    else
    {
        fprintf(stderr, "Device at %s does not support CommFPGA\n", vp);
        FAIL_RET(29, cleanup);
    }

    retVal = 0;
  cleanup:
    if (error)
    {
        fprintf(stderr, "%s\n", error);
        flFreeError(error);
    }
    flClose(*handle);
    return retVal;
}

/* Read register */
FLStatus fpgalink_read(struct FLContext *handle, uint8 channel, uint8 *value)
{
    FLStatus status;
    const char *error = NULL;
    status = flReadChannel(handle, channel, 1, value, &error);
    if (error)
    {
        fprintf(stderr, "%s\n", error);
        flFreeError(error);
        return status;
    }
    if (status != FL_SUCCESS)
    {
        fprintf(stderr, "FPGALink read returned %d\n", status);
    }
    return status;
}

/* Write register */
FLStatus fpgalink_write(struct FLContext *handle, uint8 channel, uint8 value)
{
    FLStatus status;
    const char *error = NULL;
    status = flWriteChannel(handle, channel, 1, &value, &error);
    if (error)
    {
        fprintf(stderr, "%s\n", error);
        flFreeError(error);
        return status;
    }
    if (status != FL_SUCCESS)
    {
        fprintf(stderr, "FPGALink write returned %d\n", status);
    }
    return status;
}
