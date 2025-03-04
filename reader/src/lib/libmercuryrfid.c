#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <inttypes.h>

#include "tm_reader.h"
#include "queue.h"

int readerCount = 0;
int uniqueReaderInstance = 0;
TMR_Reader ** readers = NULL;
TMR_ReadPlan** plan = NULL;
TMR_ReadListenerBlock** rlb = NULL;
TMR_ReadExceptionListenerBlock** reb = NULL;
TMR_Region** region = NULL;
TMR_Status** status = NULL;
TMR_String** model = NULL;

int16_t antennaMaxPower;
int16_t antennaMinPower;

typedef void (*ForeignCallback)(char* message);
ForeignCallback foreignCallback;

void callback(TMR_Reader *reader, const TMR_TagReadData *t, void *cookie);
int checkError(TMR_Status status, const char* msg);
int getEnum(const char* string);

typedef int ReaderID;
void* communicatorThreadFunction();
Queue* communicatorQueue;

int RFIDinit(ForeignCallback callbackHandle)
{
    foreignCallback = callbackHandle;

    communicatorQueue = createQueue();
    pthread_t communicatorThread;
    pthread_create(&communicatorThread, NULL, communicatorThreadFunction, (void*) communicatorQueue);

    return 0;
}

int RFIDclose()
{
    Enqueue(communicatorQueue, "exit");

    int i;

    for(i = 0; i < readerCount; i++)
    {
        TMR_stopReading(readers[i]);
        TMR_destroy(readers[i]);
        free(readers[i]);
        free(plan[i]);
        free(rlb[i]);
        free(reb[i]);
        free(region[i]);
        free(status[i]);
        free(model[i]);
    }

    free(readers);
    free(plan);
    free(rlb);
    free(reb);
    free(region);
    free(status);
    free(model);

    return 0;
}

void* communicatorThreadFunction(void* messageQueue)
{
    int isThreadRunning = 1;

    while(isThreadRunning)
    {
        char* message = Dequeue(messageQueue);

        if(message != NULL)
        {
            if(strcmp(message, "exit") == 0)
            {
                isThreadRunning = 0;
            }
            else
            {
                ////////////////////////////////////////////////////////////

                //break apart message, send data to correct callback
                //make sure if callback is not yet initialised,
                //that the message is preserved(possibly stuck back onto queue
                foreignCallback(message);
            }
        }
        else
        {
            int milliseconds = 2;
            usleep(1000*milliseconds);
        }
    }

    return NULL;
}

void tagCallback(TMR_Reader *readerr, const TMR_TagReadData *t, void *cookie)
{
    int i;
    int readerLocation;

    for(i = 0; i < uniqueReaderInstance; i++)
    {
        if(readers[i] == readerr)
        {
            readerLocation = i;
        }
    }

    char tagID[128];
    TMR_bytesToHex(t->tag.epc, t->tag.epcByteCount, tagID);

    char msg[256];
    sprintf(msg, "%i:%s:%d:%i:%i:%u:%u", readerLocation, tagID, t->rssi, t->phase, t->frequency, t->timestampHigh, t->timestampLow);

    //uint64_t timestamp = ((uint64_t) t->timestampHigh<< 32 ) | t->timestampLow;

    Enqueue(communicatorQueue, msg);
}

int readOnce(int readerId,int timeoutMili, int *numTags, char * tagData )
{   
      int error;
      TMR_TagReadData trd;
      char tagID[128];		         
      TMR_Status laststatus =TMR_SUCCESS;



      error=checkError(TMR_read( readers[readerId], timeoutMili, numTags) , "reading once");
  //    printf("Detected %i tags\n", *numTags);

      if (numTags>0){
// asume python provides memory
//        tagData = malloc( 256 * sizeof(char) * (*numTags) );
//        printf("address is:%p\n",tagData);
	while ( TMR_hasMoreTags(readers[readerId])==TMR_SUCCESS ){
	 	laststatus = TMR_getNextTag(readers[readerId], &trd);
		if (laststatus==TMR_SUCCESS){
			TMR_bytesToHex(trd.tag.epc, trd.tag.epcByteCount, tagID);
			sprintf(tagData, "%s%i:%s:%d:%i:%i:%u:%u:%d;", tagData,readerId, tagID, trd.rssi, trd.phase, trd.frequency, trd.timestampHigh, trd.timestampLow,trd.readCount);	
                        //printf(".");
		} //if
	} //while
//        printf("Tag data is: \n%s\n", tagData);

      } //if

return error;


}

void exceptionCallback(TMR_Reader *reader, TMR_Status error, void *cookie)
{
    fprintf(stderr, "Error:%s\n", TMR_strerr(reader, error));
}

int RFIDstartReader(const char* deviceURI)
{
    readers = realloc(readers, (readerCount + 1) * sizeof(TMR_Reader*));
    TMR_Reader* pr = malloc(sizeof(TMR_Reader));
    readers[readerCount] = pr;

    plan = realloc(plan, (readerCount + 1) * sizeof(TMR_ReadPlan*));
    TMR_ReadPlan* prp = malloc(sizeof(TMR_ReadPlan));
    plan[readerCount] = prp;

    rlb = realloc(rlb, (readerCount + 1) * sizeof(TMR_ReadListenerBlock*));
    TMR_ReadListenerBlock* prlb = malloc(sizeof(TMR_ReadListenerBlock));
    rlb[readerCount] = prlb;

    reb = realloc(reb, (readerCount + 1) * sizeof(TMR_ReadExceptionListenerBlock*));
    TMR_ReadExceptionListenerBlock* preb = malloc(sizeof(TMR_ReadExceptionListenerBlock));
    reb[readerCount] = preb;

    region = realloc(region, (readerCount + 1) * sizeof(TMR_Region*));
    TMR_Region* preg = malloc(sizeof(TMR_Region));
    region[readerCount] = preg;

    status = realloc(status, (readerCount + 1) * sizeof(TMR_Status*));
    TMR_Status* pstat = malloc(sizeof(TMR_Status));
    status[readerCount] = pstat;

    model = realloc(model, (readerCount + 1) * sizeof(TMR_String*));
    TMR_String* pstr = malloc(sizeof(TMR_String));
    model[readerCount] = pstr;

    if(checkError(TMR_create(readers[readerCount], deviceURI), "Creating reader"))
    {
        return -1;
    }
    if(checkError(TMR_connect(readers[readerCount]), "Connecting to reader"))
    {
        return -1;
    }

    *(region[readerCount]) = TMR_REGION_NONE;
    if(checkError(TMR_paramGet(readers[readerCount], TMR_PARAM_REGION_ID, region[readerCount]), "Getting Saved Region"))
    {
        return -1;
    }

    if(*(region[readerCount]) == TMR_REGION_NONE)
    {
        printf("No saved regions\n");

        TMR_RegionList regions;
        TMR_Region _regionStore[32];
        regions.list = _regionStore;
        regions.max = sizeof(_regionStore)/sizeof(_regionStore[0]);
        regions.len = 0;

        if(checkError(TMR_paramGet(readers[readerCount], TMR_PARAM_REGION_SUPPORTEDREGIONS, &regions), "Getting List of Regions"))
        {
            return -1;
        }

        if(regions.len < 1)
        {
            printf("Reader doesn't support any regions\n");
            return -1;
        }

        *(region[readerCount]) = regions.list[0];
        if(checkError(TMR_paramSet(readers[readerCount], TMR_PARAM_REGION_ID, &*(region[readerCount])), "Setting region"))
        {
            return -1;
        }
    }

    if(checkError(TMR_RP_init_simple(plan[readerCount], 0x0, NULL, TMR_TAG_PROTOCOL_GEN2, 1000), "Initialising read plan"))
    {
        return -1;
    }

    if(checkError(TMR_paramSet(readers[readerCount], TMR_PARAM_READ_PLAN, plan[readerCount]), "Setting Read Plan"))
    {
        return -1;
    }

    rlb[readerCount]->listener = tagCallback;
    rlb[readerCount]->cookie = NULL;

    reb[readerCount]->listener = exceptionCallback;
    reb[readerCount]->cookie = NULL;

    if(checkError(TMR_addReadListener(readers[readerCount], rlb[readerCount]), "Adding read listener"))
    {
        return -1;
    }

    if(checkError(TMR_addReadExceptionListener(readers[readerCount], reb[readerCount]), "Adding Exception Listener"))
    {
        return -1;
    }

    if(checkError(TMR_startReading(readers[readerCount]), "Starting reader"))
    {
        return -1;
    }

    uniqueReaderInstance++;

    return (uniqueReaderInstance - 1);
}

int RFIDstopReader(ReaderID readerID)
{
    int error;
    error = checkError(TMR_stopReading(readers[readerID]), "Stopping Reader");

    return error;
}

int reStartReader(int readerId)
{   
      int error;
      error=checkError(TMR_startReading(readers[readerId]), "restarting reader");
      return error;
}





int checkError(TMR_Status status, const char* msg)
{
    if(status == TMR_SUCCESS)
    {
        printf("%s: Ok\n", msg);
        return 0;
    }
    printf("%s: Failed - %s\n", msg, TMR_strerror(status));
    return 1;
}

void getPower(ReaderID readerID)
{
    int16_t max, min;
    int32_t current;

    checkError(TMR_stopReading(readers[readerID]), "Stopping Reader for power reading");

    TMR_paramGet(readers[readerID], TMR_PARAM_RADIO_POWERMAX , &max);
    TMR_paramGet(readers[readerID], TMR_PARAM_RADIO_POWERMIN , &min);
    TMR_paramGet(readers[readerID], TMR_PARAM_RADIO_READPOWER , &current);

    printf("Max transmission power is: %i\n", max);
    printf("Min transmission power is: %i\n", min);
    printf("Current transmission power is: %i\n", current);

    checkError(TMR_startReading(readers[readerID]), "Starting reader");
}

int setPower(ReaderID readerID, int value)
{
    //error handling on start/stop to do
    int hasError = 0;
    printf("Setting read power\n");

    hasError=checkError(TMR_stopReading(readers[readerID]), "Stopping Reader for power reading");
    hasError=checkError(TMR_paramSet(readers[readerID], TMR_PARAM_RADIO_READPOWER, &value), "Setting Radio Power");
    if (!hasError)
    {
         hasError=checkError(TMR_startReading(readers[readerID]), "Starting reader");
    }
    return hasError;



/*
    checkError(TMR_stopReading(readers[readerID]), "Stopping Reader for power reading");
    if(checkError(TMR_paramSet(readers[readerID], TMR_PARAM_RADIO_READPOWER, &value), "Setting Radio Power"))
    {
        return -1;
    }
    checkError(TMR_startReading(readers[readerID]), "Starting reader");
    return hasError;
*/
}

 
int setRegionEU(int readerID)
{    
    TMR_Region region;    
    int hasError;
    region=TMR_REGION_EU3; //see tmr_params.h for other regions. Not all are supported.
    
    hasError=checkError(TMR_stopReading(readers[readerID]), "Stopping Reader for EU region configuration");      
    if (!hasError)
    {
        hasError=checkError(TMR_paramSet(readers[readerID], TMR_PARAM_REGION_ID, &region), "Setting EU region");
    }
    
    if (!hasError)
    {
         hasError=checkError(TMR_startReading(readers[readerID]), "Starting reader");
    }
    
    return hasError;
}


uint8_t castValue(char ascii)
{
    uint8_t val;
    if ((ascii<='9')&&(ascii>='0'))
    {
        val=ascii-'0';
    } 
    else if ((ascii<='F')&&(ascii>='A'))
    {
        val=ascii-'A'+10;
    }
    else if ((ascii<='f')&&(ascii>='a'))
    {
        val=ascii-'a'+10;
    }
    return val;
}

/**
 * Char based Version, needs reader to be stop and restarted
 * */
int writeTag(ReaderID readerId, char epcData[], uint8_t epcBytes)
{
    int error;
    TMR_TagData epc;
    TMR_TagOp tagop;

    uint8_t epcDataB[epcBytes];

    int i;
    printf("New tag id to write is: [");
    for (i=0;i<epcBytes;i++)
    {
    printf("0x%c%c",epcData[2*i],epcData[2*i+1]);
    if (i<epcBytes-1) printf(", ");
    }
    printf("]\n");

    for (i=0;i<epcBytes;i++)
    {
    uint8_t highB=castValue(epcData[2*i]);  
    uint8_t lowB=castValue(epcData[2*i+1]);
    epcDataB[i]=(highB<<4)|lowB;
     }

     printf("binary value is: [");
     for (i=0;i<epcBytes;i++)
    {
        printf("0x%02X",epcDataB[i]);
        if (i<11) printf(", ");
    }
    printf("]\n");


    
    //epc.epcByteCount = sizeof(epcData) / sizeof(epcData[0]);
    epc.epcByteCount = epcBytes;
    memcpy(epc.epc, epcDataB, epc.epcByteCount * sizeof(uint8_t));

    error=checkError(TMR_TagOp_init_GEN2_WriteTag(&tagop, &epc), "initializing GEN2_WriteTag");
    error=checkError(TMR_executeTagOp(readers[readerId], &tagop, NULL, NULL), "executing GEN2_WriteTag");

    if (!error)
    {
            printf("Success..........................................................\n");
    } 
    else
    {
        printf("CANNOT set tag to known value\n");      
    }
    

    return error;
  }



/**
 * Changes epc code from epcDataOLD to epcData from a tag.
 * Reader must be stopped and restarted before using it.
 * We assume both epcData epcDataOLD have same lenght
 * */
int renameTag(ReaderID readerId, char epcDataOLD[], char epcData[], uint8_t epcBytes)
{
    int error;
    TMR_TagData epc;
    TMR_TagData epcOLD;

    TMR_TagOp tagop;
    TMR_TagFilter filter;
    
    uint8_t epcDataB[epcBytes];
    uint8_t epcDataOLDB[epcBytes];
    uint8_t highB;  
    uint8_t lowB;
    
    int i;
    
    // cast char values to binary values
    for (i=0;i<epcBytes;i++)
    {
      highB=castValue(epcData[2*i]);    
      lowB=castValue(epcData[2*i+1]);
      epcDataB[i]=(highB<<4)|lowB;
      
      highB=castValue(epcDataOLD[2*i]); 
      lowB=castValue(epcDataOLD[2*i+1]);
      epcDataOLDB[i]=(highB<<4)|lowB;
      
     }

    // Initialize the filter with the original tag epc code
    epcOLD.epcByteCount = epcBytes;
    memcpy(epcOLD.epc, epcDataOLDB, epcOLD.epcByteCount * sizeof(uint8_t));  
    error=checkError(TMR_TF_init_tag(&filter, &epcOLD), "initializing TMR_TagFilter");

    // Initialize tag write operation  
    epc.epcByteCount = epcBytes;
    memcpy(epc.epc, epcDataB, epc.epcByteCount * sizeof(uint8_t));  
    error=checkError(TMR_TagOp_init_GEN2_WriteTag(&tagop, &epc), "initializing GEN2_WriteTag");    
    
    // Execute tag writting with filter
    error=checkError(TMR_executeTagOp(readers[readerId], &tagop, &filter, NULL), "executing GEN2_WriteTag");

    if (!error)
    {
            printf("Success..........................................................\n");
    } 
    else
    {
        printf("CANNOT set tag to known value\n");      
    }
    

    return error;
  }


/**
 * Unchecked version, needs reader to be stop and restarted
 * */
int writeTag2(ReaderID readerId, uint8_t epcData[], uint8_t epcBytes)
{
    int error;
    TMR_TagData epc;
    TMR_TagOp tagop;


    int i;
    printf("New tag id to write is: [");
    for (i=0;i<12;i++)
    {
        printf("0x%02X",epcData[i]);
        if (i<11) printf(", ");
    }
    printf("]\n");

    //epc.epcByteCount = sizeof(epcData) / sizeof(epcData[0]);
    epc.epcByteCount = epcBytes;
    memcpy(epc.epc, epcData, epc.epcByteCount * sizeof(uint8_t));

    error=checkError(TMR_TagOp_init_GEN2_WriteTag(&tagop, &epc), "initializing GEN2_WriteTag");
    error=checkError(TMR_executeTagOp(readers[readerId], &tagop, NULL, NULL), "executing GEN2_WriteTag");

    if (!error)
    {
            printf("Success..........................................................\n");
    } 
    else
    {
        printf("CANNOT set tag to known value\n");
    }
    

    return error;
}


 
 
 /*
  * First version, based on api samples. Safer one
  * 
  * 
  **/
int writeTagOLD(ReaderID readerId, uint8_t newEpcData[], uint8_t epcBytes)
{
    int error;
    uint8_t epcData[] = {
      0x01, 0x23, 0x45, 0x67, 0x89, 0xAB,
      0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67,
      };
    TMR_TagData epc;
    TMR_TagOp tagop;

    error=checkError(TMR_stopReading(readers[readerId]), "Stopping reader before tag writting");


    /* Set the tag EPC to a known value*/
    epc.epcByteCount = sizeof(epcData) / sizeof(epcData[0]);
    memcpy(epc.epc, epcData, epc.epcByteCount * sizeof(uint8_t));

    error=checkError(TMR_TagOp_init_GEN2_WriteTag(&tagop, &epc), "initializing GEN2_WriteTag");
    error=checkError(TMR_executeTagOp(readers[readerId], &tagop, NULL, NULL), "executing GEN2_WriteTag");

    if (!error)
    {
            printf("Tag has now a known value................................................................\n");
    } else
    {
        printf("CANNOT set tag to known value\n");
        return error;       
    }
    
    int i;
    printf("New tag id to write is: [");
    for (i=0;i<12;i++)
    {
        printf("0x%02X",newEpcData[i]);
        if (i<11) printf(", ");
    }
    printf("]\n");

    if (!error)
    {  /* Write Tag EPC with a select filter*/    
      TMR_TagFilter filter;
      TMR_TagData newEpc;
      TMR_TagOp newtagop;

      // This should work... but it does not
      //newEpc.epcByteCount = sizeof(newEpcData) / sizeof(newEpcData[0]);
      newEpc.epcByteCount = epcBytes;
      memcpy(newEpc.epc, newEpcData, newEpc.epcByteCount * sizeof(uint8_t));
      
      printf("We are about to write %d bytes \n",newEpc.epcByteCount);
      //printf("Size of newEpcData %lu\n",sizeof(newEpcData) );
      printf("Size of newEpcData[0] %lu\n", sizeof(newEpcData[0]));
      
      
      /* Initialize the new tagop to write the new epc*/       
      error=checkError(TMR_TagOp_init_GEN2_WriteTag(&newtagop, &newEpc), "initializing GEN2_WriteTag");

          /* Initialize the filter with the original epc of the tag which is set earlier*/
      error=checkError(TMR_TF_init_tag(&filter, &epc), "initializing TMR_TagFilter");

      /* Execute the tag operation Gen2 writeTag with select filter applied*/
      error=checkError(TMR_executeTagOp(readers[readerId], &newtagop, &filter, NULL), "executing GEN2_WriteTag");
      
      error=checkError(TMR_startReading(readers[readerId]), "Starting reader");

    }
    return error;
  }

void getHopTime(ReaderID readerID)
{
    checkError(TMR_stopReading(readers[readerID]), "Stopping reader for hop time reading");

    uint32_t hoptimeval;

    TMR_paramGet(readers[readerID], TMR_PARAM_REGION_HOPTIME, &hoptimeval);
    //TMR_paramGet(readers[readerID], TMR_PARAM_REGION_HOPTABLE, &);

    printf("HOPTIME: %" PRIu32  "\n", hoptimeval);
    //printf("HOPTABLE: \n");

    checkError(TMR_startReading(readers[readerID]), "Starting Reader");

}

void setHopTime(ReaderID readerID, int value)
{
    uint32_t hoptime = (uint32_t) value;

    printf("New hoptime is: %" PRIu32 "\n", hoptime);

    checkError(TMR_stopReading(readers[readerID]), "Stopping reader for hop time reading");

    checkError(TMR_paramSet(readers[readerID], TMR_PARAM_REGION_HOPTIME, &hoptime), "Setting hoptime");

    checkError(TMR_startReading(readers[readerID]), "Starting reader");

}

int setParameter(const char* parameterS, const char* value)
{
    return 0;
    /*int parameter = getEnum(parameterS);

    if(checkError(TMR_paramSet(readerp, parameter, value), "Setting External Parameter"))
    {
        return -1;
    }
    else
    {
        return 0;
    }*/
}

/*
void* getParameter(const char* parameterS
{
    int parameter = getEnum(parameterS);

    if(checkError(TMR_paramSet(readerp, parameter,


}*/
