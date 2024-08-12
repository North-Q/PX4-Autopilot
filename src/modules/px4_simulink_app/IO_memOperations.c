/**
* @file IO_memOperations.c
*
* Contains definitions of memoryread and memory write functions.
*
* @Copyright 2023 The MathWorks, Inc.
*
*/
#include "peripheralIncludes.h"
#include "IO_memOperations.h"
#include "IO_packet.h"

#ifndef CHARSIZE
    #define MEMORY_ADJUSTMENT 1
#else
    #ifndef MEMORY_ADJUSTMENT
        #define MEMORY_ADJUSTMENT (CHARSIZE/8)
    #endif
#endif

/*
* The following code is used for debugging purpose
* The debug messages print the function in which it is called and the memory address to be modified
*/
#if DEBUG_FLAG == 1

#define WRITE    "memoryWrite::Address %" PRIu32 ";"
#define BITWRITE "memoryBitWrite::Address %" PRIu32 ";"
#define READ     "memoryRead::Address %" PRIu32 ";"

#ifdef __AVR_ARCH__
const char MSG_MEMORYWRITE[] PGMKEYWORD = WRITE;
const char MSG_MEMORYBITWRITE[] PGMKEYWORD = BITWRITE;
const char MSG_MEMORYREAD[] PGMKEYWORD = READ;
#else
const char MSG_MEMORYWRITE[] = WRITE;
const char MSG_MEMORYBITWRITE[] = BITWRITE;
const char MSG_MEMORYREAD[] = READ;
#endif

#endif

#ifndef __AVR_ARCH__
/**
  * @brief  Packs the data passed as arguments into 64-bit value
  * @param  data - Pointer to data to be packed
  * @retval uint16_T
  */
uint64_T packBytesInto64BitInteger(uint8_T* data){
    return (uint64_T) (((uint64_T)data[0]) | (((uint64_T)data[1])<<8) | (((uint64_T)data[2])<<16) | (((uint64_T)data[3])<<24) | (((uint64_T)data[4])<<32) | (((uint64_T)data[5])<<40) | (((uint64_T)data[6])<<48) | (((uint64_T)data[7])<<56));
}

/**
  * @brief  Split the value passed as arguments into 8-bit bytearray
  * @param  value - Value to be split into 8-bit bytearray
  * @param   data - Pointer to data to be packed
  * @retval None
  */
void unpack64BitstoByte(uint64_T value, uint8_T *data) { 
    uint32_T datauint32 = value & 0xFFFFFFFF;
    unpack32BitstoByte(datauint32,&data[0]);
    datauint32 = (value>>32) & 0xFFFFFFFF;
    unpack32BitstoByte(datauint32,&data[4]);
}
#endif

/**
* @brief  Get address bus size information of the microcontroller
* @param             payloadBufferTx - Pointer to transport payload
* @param  peripheralDataSizeResponse - Pointer to data payload size
* @param                      status - Pointer to status
* @retval None
*/
void getBusWidth(uint8_T *payloadBufferTx, payloadSize_T *peripheralDataSizeResponse, uint8_T *status)
{
//If get address bus width is not defined the value 0 will be transmitted to host and status will be sent as 1
#ifndef ADDRESSBUSWIDTH
    uint8_T addressBusWidth = 0;
    *status = 1;
#else
    uint8_T addressBusWidth = (uint8_T)ADDRESSBUSWIDTH;
    *status = 0;
#endif

#ifndef CHARSIZE
    uint8_T charSize = 0;
    *status = 1;
#else
    uint8_T charSize = (uint8_T) CHARSIZE;
    *status = *status?1:0;
#endif

    memcpy(payloadBufferTx,&addressBusWidth,sizeof(uint8_T));
    memcpy(&payloadBufferTx[1],&charSize,sizeof(uint8_T));
    *peripheralDataSizeResponse = 2;  
}

/**
* @brief  Writes the data received from the host into the specified memory location
* @param  payloadBufferRx - Pointer to peripheralPayload
* @retval None
*/
void memoryWrite(uint8_T* payloadBufferRx) {

    void *memoryLocation = 0;                                   // Temporarily stores the memory location extracted from the peripheralPayload
    uint8_T idx = 0;                                            // Track the index position of peripheralPayload
    uint16_T numberOfBytesToWrite = 0;                           // Get data length from the peripheral payload
    uint8_T disableMemoryProtection = payloadBufferRx[idx++];   // Flag to execute target specific protection disable function
    uint32_T numberOfData = 0;                                  // Bytes to write into the target



    /* Extract memory address from payloadBuffer */
    #if (ADDRESSBUSWIDTH == 8)
    memoryLocation = (void *) (uint8_T)(payloadBufferRx[idx]);
    #elif (ADDRESSBUSWIDTH == 16)
    memoryLocation = (void *) packBytesInto16BitInteger(&payloadBufferRx[idx]);
    #elif (ADDRESSBUSWIDTH == 32)
    memoryLocation = (void *) packBytesInto32BitInteger(&payloadBufferRx[idx]);
    #ifndef __AVR_ARCH__
    #elif (ADDRESSBUSWIDTH == 64)
    memoryLocation = (void *) packBytesInto64BitInteger(&payloadBufferRx[idx]);
    #endif
    #endif

    #if DEBUG_FLAG == 1
    /* Prints a debug message */
    debugPrint(MSG_MEMORYWRITE,memoryLocation);
    #endif

    /* Compute the number of bytes to write into the target */
    idx = idx + (ADDRESSBUSWIDTH/8);                               //Increment pointer index to fetch the next value
    numberOfBytesToWrite = packBytesInto16BitInteger(&payloadBufferRx[idx]); //Extract the data length sent in the peripheral payload
    idx += 2;

    //If target is of data bus size 8-bit memcpy(,,1) will copy 1 bytes
    //If target is of data bus size 16-bit memcpy(,,1) will copy 2 bytes
    //If target is of data bus size 32-bit memcpy(,,1) will copy 4 bytes.
    //So, the 8-bit stream of data in the peripheralPayload is grouped before writing into the target

    //Calculate the number of bytes to be written into the target based on data bus size of the target
    numberOfData = (numberOfBytesToWrite/MEMORY_ADJUSTMENT);

    uint16_T index = idx;

    /* Arrange the data based on target data bus size */
    #if (CHARSIZE == 16)
    uint16_T cnt;
    for (cnt = 0; cnt<numberOfData; cnt++){
        payloadBufferRx[index+cnt] = packBytesInto16BitInteger(&payloadBufferRx[idx]);   //for data bus width of 16 the payloadBufferRx is a 16bit integer array
        idx = idx + 2;
    }
    #elif (CHARSIZE == 32)
    uint16_T cnt;
    for (cnt = 0; cnt<numberOfData; cnt++){
        payloadBufferRx[index+cnt] = packBytesInto32BitInteger(&payloadBufferRx[idx]);   //for data bus width of 32 the payloadBufferRx is a 32bit integer array
        idx = idx + 4;
    }
    #elif (CHARSIZE == 64)
    uint16_T cnt;
    for (cnt = 0; cnt<numberOfData; cnt++){
        payloadBufferRx[index+cnt] = packBytesInto64BitInteger(&payloadBufferRx[idx]);   //for data bus width of 64 the payloadBufferRx is a 64bit integer array
        idx = idx + 8;
    }
    #endif

    /*Write data into memory location*/
    //Since some targets may not have memory protection, the disableMemoryProtection and enableMemoryProtection functions may not be
    //declared by target authors so they are placed under gaurd

    #if IO_STANDARD_MEMORY_OPERATIONS_PROTECTION_AVAILABLE
    if ( disableMemoryProtection )
    {
        disableMemoryProtectionFunction(payloadBufferRx); // Disable memory protection before updating the memory
    }
    #endif
    memcpy(memoryLocation,&payloadBufferRx[index],numberOfData);
    #if IO_STANDARD_MEMORY_OPERATIONS_PROTECTION_AVAILABLE
    if ( disableMemoryProtection )
    {
        enableMemoryProtectionFunction(payloadBufferRx); // Enable memory protection after updating the memory
    }
    #endif

}

/**
* @brief  Writes the data received from the host into the specified bit position of the memory location
* @param  payloadBufferRx - Pointer to peripheralPayload
* @retval None
*/

void memoryBitWrite(uint8_T* payloadBufferRx) {
    void *memoryLocation = 0;               // Temporarily stores the memory location extracted from the peripheralPayload
    uint8_T idx = 0;                        // Track the index position of peripheralPayload
    uint16_T numberOfBytesToWrite;           // Bytes to write into the target

    uint8_T disableMemoryProtection = payloadBufferRx[idx++];   // Flag to execute target specific protection disable function

    /* Extract memory address from payloadBuffer */
    #if (ADDRESSBUSWIDTH == 8)
    memoryLocation = (void *) (uint8_T)(payloadBufferRx[idx]);
    #elif (ADDRESSBUSWIDTH == 16)
    memoryLocation = (void *) packBytesInto16BitInteger(&payloadBufferRx[idx]);
    #elif (ADDRESSBUSWIDTH == 32)
    memoryLocation = (void *) packBytesInto32BitInteger(&payloadBufferRx[idx]);
    #elif (ADDRESSBUSWIDTH == 64)
    memoryLocation = (void *) packBytesInto64BitInteger(&payloadBufferRx[idx]);
    #endif

    #if DEBUG_FLAG == 1
    /* Prints a debug message */
    debugPrint(MSG_MEMORYBITWRITE,memoryLocation);
    #endif

    /* Compute the number of bytes to write into the target */
    idx = idx + (ADDRESSBUSWIDTH/8);                //Increment pointer index to fetch the next value
    numberOfBytesToWrite = packBytesInto16BitInteger(&payloadBufferRx[idx]); //Extract the data length sent in the peripheral payload
    idx += 2;

    /*Write data into memory location*/
    //Since some targets may not have memory protection, the disableMemoryProtection and enableMemoryProtection functions may not be
    //declared by target authors so they are placed under gaurd
    #if IO_STANDARD_MEMORY_OPERATIONS_PROTECTION_AVAILABLE
    if ( disableMemoryProtection )
    {
        disableMemoryProtectionFunction(payloadBufferRx); // Disable memory protection before updating the memory
    }
    #endif

    /*
    * Write logic
    * Extract data from the memory location
    * AND it with the bit position data coming from the host to clear the necessary bits
    *  OR it with the bit values data coming from the host to set the necessary bits
    */
    if( numberOfBytesToWrite == 1 ){
        uint8_T *tempData = (uint8_T *)memoryLocation;
        uint8_T bitPosition = payloadBufferRx[idx++];
        *tempData = (*tempData & bitPosition );
        uint8_T bitValue = payloadBufferRx[idx];
        *tempData = (*tempData | bitValue );
    }
    else if( numberOfBytesToWrite == 2 ){
        uint16_T *tempData = (uint16_T *)memoryLocation;
        uint16_T bitPosition = packBytesInto16BitInteger(&payloadBufferRx[idx]);
        *tempData = (*tempData & bitPosition );
        uint16_T bitValue = packBytesInto16BitInteger(&payloadBufferRx[idx+2]);
        *tempData = (*tempData | bitValue );
    }
    else if( numberOfBytesToWrite == 4 ){
        uint32_T *tempData = (uint32_T *)memoryLocation;
        uint32_T bitPosition = packBytesInto32BitInteger(&payloadBufferRx[idx]);
        *tempData = (*tempData & bitPosition );
        uint32_T bitValue = packBytesInto32BitInteger(&payloadBufferRx[idx+4]);
        *tempData = (*tempData | bitValue );
    }
    #ifndef __AVR_ARCH__
    else if( numberOfBytesToWrite == 8 ){
        uint64_T *tempData = (uint64_T *)memoryLocation;
        uint64_T bitPosition = packBytesInto64BitInteger(&payloadBufferRx[idx]);
        *tempData = (*tempData & bitPosition );
        uint64_T bitValue = packBytesInto64BitInteger(&payloadBufferRx[idx+8]);
        *tempData = (*tempData | bitValue );
    }
    #endif
    #if IO_STANDARD_MEMORY_OPERATIONS_PROTECTION_AVAILABLE
    if ( disableMemoryProtection )
    {
        enableMemoryProtectionFunction(payloadBufferRx); // Enable memory protection after updating the memory
    }
    #endif
}

/**
* @brief  Read the data from the memory location received from the host
* @param             payloadBufferRx - Pointer to peripheral payload
* @param             payloadBufferTx - Pointer to transport payload
* @param  peripheralDataSizeResponse - Pointer to data payload size
* @retval None
*/
void memoryRead(uint8_T* payloadBufferRx, uint8_T* payloadBufferTx, uint16_T* peripheralDataSizeResponse) {
    void * memoryLocation = 0;          // Temporarily stores the memory location extracted from the peripheralPayload
    uint8_T idx = 0;                    // Track the index position of peripheralPayload
    uint16_T numberOfData = 0;          // Bytes to write into the target
    uint32_T cnt = 0;                   // Used in for loop
    uint32_T numberOfBytesToWrite;
    uint8_T disableMemoryProtection = payloadBufferRx[idx++];   // Flag to execute target specific protection disable function

    /* Extract memory address from payloadBuffer */
    #if (ADDRESSBUSWIDTH == 8)
    memoryLocation = (void *) (uint8_T)(payloadBufferRx[idx]);
    #elif (ADDRESSBUSWIDTH == 16)
    memoryLocation = (void *) packBytesInto16BitInteger(&payloadBufferRx[idx]);
    #elif (ADDRESSBUSWIDTH == 32)
    memoryLocation = (void *) packBytesInto32BitInteger(&payloadBufferRx[idx]);
    #elif (ADDRESSBUSWIDTH == 64)
    memoryLocation = (void *) packBytesInto64BitInteger(&payloadBufferRx[idx]);
    #endif

    #if DEBUG_FLAG == 1
    /* Prints a debug message */
    debugPrint(MSG_MEMORYREAD,memoryLocation);
    #endif

    /* Compute the number of bytes to read from the target */
    //If target is of data bus size 8-bit memcpy(,,1) will copy 1 bytes
    //If target is of data bus size 16-bit memcpy(,,1) will copy 2 bytes
    //If target is of data bus size 32-bit memcpy(,,1) will copy 4 bytes.
    //Calculate the number of bytes to be written into the target based on data bus size of the target
    idx = idx + (ADDRESSBUSWIDTH/8);                            //Increment pointer index to fetch the next value
    numberOfBytesToWrite = packBytesInto16BitInteger(&payloadBufferRx[idx]); //Extract the data length sent in the peripheral payload
    numberOfData = (numberOfBytesToWrite/MEMORY_ADJUSTMENT);    //Extract the data length sent in the peripheral payload

    /*Write data into memory location*/
    //Since some targets may not have memory protection, the disableMemoryProtection and enableMemoryProtection functions may not be
    //declared by target authors so they are placed under gaurd
    #if IO_STANDARD_MEMORY_OPERATIONS_PROTECTION_AVAILABLE
    if ( disableMemoryProtection )
    {
        disableMemoryProtectionFunction(payloadBufferRx); // Disable memory protection before updating the memory
    }
    #endif

    /*
    * Logic:
    * Extract the data from the memory location specified in the peripheral payload
    * Convert the data into 8-bit stream of data before transferring it to the host
    */
    #if (CHARSIZE == 8)
    memcpy(payloadBufferTx,memoryLocation,numberOfData);
    #elif (CHARSIZE == 16)
    uint16_T tempData = 0;
    for (cnt = 0; cnt<numberOfData; cnt++){
        memcpy(&tempData,memoryLocation,sizeof(uint16_T));
        byteUnpack16Bits((payloadBufferTx + cnt*2), &tempData);
        memoryLocation = (void *)((uint16_T *)memoryLocation + sizeof(uint16_T));
    }
    #elif (CHARSIZE == 32)
    uint32_T tempData = 0;
    for (cnt = 0; cnt<numberOfData; cnt++){
        memcpy(&tempData,memoryLocation,sizeof(uint32_T));
        unpack32BitstoByte((payloadBufferTx + cnt*4),&tempData);
        memoryLocation = (void *)((uint32_T *)memoryLocation + sizeof(uint32_T));
    }
    #ifndef __AVR_ARCH__
    #elif (CHARSIZE == 64)
    uint64_T tempData = 0;
    for (cnt = 0; cnt<numberOfData; cnt++){
        memcpy(&tempData,memoryLocation,sizeof(uint64_T));
        unpack64BitstoByte((payloadBufferTx + cnt*8),&tempData);
        memoryLocation = (void *)((uint64_T *)memoryLocation + sizeof(uint64_T));
    }
    #endif
    #endif
    
    #if IO_STANDARD_MEMORY_OPERATIONS_PROTECTION_AVAILABLE
    if ( disableMemoryProtection )
    {
        enableMemoryProtectionFunction(payloadBufferRx); // Enable memory protection before updating the memory
    }
    #endif

    // Send the number of bytes to read value back to the host
    *peripheralDataSizeResponse = packBytesInto16BitInteger(&payloadBufferRx[idx]);
}
