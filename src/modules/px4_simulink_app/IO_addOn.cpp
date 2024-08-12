/**
 * @file IO_addOn.cpp
 *
 * Contains definition of standard addon function
 *
 * @Copyright 2018 The MathWorks, Inc.
 *
 */
#include "IO_addOn.h"
#include "hardware.h"
#if ADD_ON
#include "addOnLibraries.h"
#endif


void addon(uint8_T* payloadBufferRx, uint8_T* payloadBufferTx, uint16_T* peripheralDataSizeResponse, uint8_T* status)
{
#if ADD_ON
    uint8_T libraryIndex = 0;
    uint8_T commandId = 0;
    unsigned int payloadsize = 0;
    uint8_T* payloadBufferRxForLib = NULL;
    
    uint32_T tempPayLoadSize = 0;    // Calculates the 32 bit payload size value received from host
    LibraryBase* activeLibrary = NULL;
    uint8_T index = 0;
    
    libraryIndex = payloadBufferRx[index++];     // Indicates which add-on the user is refering to
    commandId = (uint8_T)payloadBufferRx[index++];    // Command Id for a particular library
    memcpy(&tempPayLoadSize, &payloadBufferRx[index], sizeof(uint32_T));   // Read four bytes to as the size of add-on payload
    index += sizeof(uint32_T);
    
    /* Define local buffer and use pointer to it to avoid alignment issues */
    uint32_T localBuffer[(tempPayLoadSize >> 2) + 1];
    memcpy(localBuffer, &payloadBufferRx[index], tempPayLoadSize);
    payloadBufferRxForLib = (uint8_T *)localBuffer;      //Receive payload buffer
    
    payloadsize = (unsigned int)tempPayLoadSize;
    
    if(libraryIndex >= hwObject.noOfLibraries)
    {
        /*Error condition
         */
        *status = uint8_T(1);
        return;
    }
    activeLibrary = hwObject.arrayOfLibraries[libraryIndex];
    if(activeLibrary == NULL)
    {
        /*Error condition
         */
        *status = uint8_T(2);
        return;
    }
    activeLibrary->commandHandler(commandId, payloadBufferRxForLib, payloadsize);
#endif
}