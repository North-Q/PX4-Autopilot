/**
 * @file IO_wrapperSCI.c
 *
 * Wraps around SCI SVD C layer to provide features to access Serial pins through IO Server
 *
 * @Copyright 2019-2020 The MathWorks, Inc.
 *
 */

#include "IO_wrapperSCI.h"
#if IO_STANDARD_SCI

#if DEBUG_FLAG 
// To DO
#endif

#ifdef USE_BIT_FOR_HANDLE
/*
Creates bits to convey info whether pin is occupied. Bit position is pin no
ex - 33 pins.
33/8 = 4 
33%8 = 1
Provide 5 bytes data to hold pin info
if pin 33 is set then 
byte 5, position 0 is set to 1
*/

/* if 16 modules are available, 16/8, 2 bytes are required, if 17 modules are available, 17/8 + 1, 3 bytes are required */ 
PeripheralHandleMapType sciMap[(IO_SCI_MODULES_MAX/8)+((IO_SCI_MODULES_MAX%8)!=0)] = {0};
#else
PeripheralHandleMapType sciMap[IO_SCI_MODULES_MAX] = {NULL};
#endif

/* The IO_SCI_MODULES_MAX macros will be defined in peripheralIncludes.h */


/* Initialize a SCI */
void openSCIBus(uint8_T* payloadBufferRx, uint8_T* payloadBufferTx, uint16_T* peripheralDataSizeResponse, uint8_T* status)
{
    void* module = 0;
    uint32_T rxPin = 0;
    uint32_T txPin = 0;
    uint8_T isString = 0;
    uint8_T index = 0;
    uint8_T mapIndex = 0;
    MW_Handle_Type sciHandle = NULL;
    /* Set status as bus error by default */
    *status = (uint8_T)MW_SCI_BUS_ERROR;
    /* Read 1 byte from input buffer indicating whether serial port is a string or not */
    isString = payloadBufferRx[index++];
    /* Read 4 byte RxPin of bus from input buffer */
    memcpy(&rxPin, &payloadBufferRx[index], sizeof(uint32_T));
    index += sizeof(uint32_T);
    /* Read 4 byte TxPin of bus from input buffer */
    memcpy(&txPin, &payloadBufferRx[index], sizeof(uint32_T));
    index += sizeof(uint32_T);
    /* Read 1 byte from input buffer indicating the index of peripheral handle map */
    memcpy(&mapIndex, &payloadBufferRx[index], sizeof(uint8_T));
    index += sizeof(uint8_T);
    /* Rest of input buffer indicates Serial Port Number or Name */
    module = &payloadBufferRx[index];
    /* Opens the bus */
    sciHandle = MW_SCI_Open((void *) module, isString, rxPin, txPin);
    if ((MW_Handle_Type)NULL != sciHandle) 
    {
        /* Stores the handle if open was successful */
        setHandle((uint32_T)mapIndex, sciHandle, sciMap);
        *status = (uint8_T)MW_SCI_SUCCESS;
    }
}


/* Set SCI frame format */
void configureSCIHardwareFlowControl(uint8_T* payloadBufferRx, uint8_T* payloadBufferTx, uint16_T* peripheralDataSizeResponse, uint8_T* status)
{
    uint32_T module = 0;
    MW_SCI_HardwareFlowControl_Type hardwareFlowControl;
    uint32_T rtsDtrPin = 0;
    uint32_T ctsDtsPin = 0;
    uint8_T index = 0; 
    MW_Handle_Type sciHandle;

    /* Set status as bus error by default */
    *status = (uint8_T)MW_SCI_BUS_ERROR;

    /* Read 4 byte Module of bus from input buffer */
    memcpy(&module, &payloadBufferRx[index], sizeof(uint32_T));
    index += sizeof(uint32_T);
    /* Read 1 byte MW_SCI_HardwareFlowControl_Type of bus from input buffer */
    hardwareFlowControl = payloadBufferRx[index++];
    /* Read 4 byte RxPin of bus from input buffer */
    memcpy(&rtsDtrPin, &payloadBufferRx[index], sizeof(uint32_T));
    index += sizeof(uint32_T);
    /* Read 4 byte TxPin of bus from input buffer */
    memcpy(&ctsDtsPin, &payloadBufferRx[index], sizeof(uint32_T));
    index += sizeof(uint32_T);
    /* Retrieve the SCI bus handle from the corresponding handle map */
    sciHandle = getHandle(module, sciMap);
    if ((MW_Handle_Type)NULL != sciHandle) 
    {
        /* configure hardware flow control */
        *status = (uint8_T)MW_SCI_ConfigureHardwareFlowControl((MW_Handle_Type)sciHandle,(MW_SCI_HardwareFlowControl_Type)hardwareFlowControl,rtsDtrPin,ctsDtsPin);
    }
}

#ifdef LTC_BAREMETAL_HARDWARE
void setSCITimeOut(uint8_T* payloadBufferRx, uint8_T* payloadBufferTx, uint16_T* peripheralDataSizeResponse, uint8_T* status)
{
    uint32_T module = 0;
    uint32_T timeOut = 0;
    uint8_T index = 0; 
    MW_Handle_Type sciHandle;

    /* Set status as bus error by default */
    *status = (uint8_T)MW_SCI_BUS_ERROR;

    /* Read 4 byte Module of bus from input buffer */
    memcpy(&module, &payloadBufferRx[index], sizeof(uint32_T));
    index += sizeof(uint32_T);
    /* Read 4 byte Timeout of bus from input buffer */
    memcpy(&timeOut, &payloadBufferRx[index], sizeof(uint32_T));
    index += sizeof(uint32_T);
    /* Retrieve the SCI bus handle from the corresponding handle map */
    sciHandle = getHandle(module, sciMap);
    if ((MW_Handle_Type)NULL != sciHandle) 
    {
        /* Set SCI timeout*/
        *status = (uint8_T)MW_SCI_ConfigureTimeOut(sciHandle, timeOut);
    }
}
#endif

/* Set the SCI bus speed */
void setSCIBaudrate(uint8_T* payloadBufferRx, uint8_T* payloadBufferTx, uint16_T* peripheralDataSizeResponse, uint8_T* status)
{
    uint32_T module = 0;
    uint32_T baudRate = 0;
    uint8_T index = 0; 
    MW_Handle_Type sciHandle;

    /* Set status as bus error by default */
    *status = (uint8_T)MW_SCI_BUS_ERROR;

    /* Read 4 byte Module of bus from input buffer */
    memcpy(&module, &payloadBufferRx[index], sizeof(uint32_T));
    index += sizeof(uint32_T);
    /* Read 4 byte Baudrate of bus from input buffer */
    memcpy(&baudRate, &payloadBufferRx[index], sizeof(uint32_T));
    index += sizeof(uint32_T);
    /* Retrieve the SCI bus handle from the corresponding handle map */
    sciHandle = getHandle(module, sciMap);
    if ((MW_Handle_Type)NULL != sciHandle) 
    {
        /* Set SCI baudrate*/
        *status = (uint8_T)MW_SCI_SetBaudrate(sciHandle, baudRate);
    }
}



/* Set SCI frame format */
void setSCIFrameFormat(uint8_T* payloadBufferRx, uint8_T* payloadBufferTx, uint16_T* peripheralDataSizeResponse, uint8_T* status)
{
    uint32_T module = 0;
    uint8_T dataBitsLength = 0;
    MW_SCI_Parity_Type parity;
    MW_SCI_StopBits_Type stopBits;
    uint8_T index = 0; 
    MW_Handle_Type sciHandle;

    /* Set status as bus error by default */
    *status = (uint8_T)MW_SCI_BUS_ERROR;

    /* Read 4 byte Module of bus from input buffer */
    memcpy(&module, &payloadBufferRx[index], sizeof(uint32_T));
    index += sizeof(uint32_T);
    /* Read 1 byte data bits length of bus from input buffer */
    dataBitsLength = payloadBufferRx[index++];
    /* Read 1 byte parity of bus from input buffer */
    parity = payloadBufferRx[index++];
    /* Read 1 byte stopBits of bus from input buffer */
    stopBits = payloadBufferRx[index++];
    /* Retrieve the SCI bus handle from the corresponding handle map */
    sciHandle = getHandle(module, sciMap);
    if ((MW_Handle_Type)NULL != sciHandle) 
    {
        /* Set SCI frame format */
        *status = (uint8_T)MW_SCI_SetFrameFormat(sciHandle, dataBitsLength, (MW_SCI_Parity_Type)parity, (MW_SCI_StopBits_Type)stopBits);
    }
}
 

/* Receive the data over SCI */
void sciReceive(uint8_T* payloadBufferRx, uint8_T* payloadBufferTx, uint16_T* peripheralDataSizeResponse, uint8_T* status)
{
    uint32_T module = 0;
    uint32_T dataLength = 0;
    uint8_T index = 0; 
    uint8_T *rxData = NULL;
    MW_Handle_Type sciHandle;

    /* Set status as bus error by default */
    *status = (uint8_T)MW_SCI_BUS_ERROR;

    /* Read 4 byte Module of bus from input buffer */
    memcpy(&module, &payloadBufferRx[index], sizeof(uint32_T));
    index += sizeof(uint32_T);
    /* Read 4 byte data length of bus from input buffer */
    memcpy(&dataLength, &payloadBufferRx[index], sizeof(uint32_T));
    index += sizeof(uint32_T);
    /* Define local buffer and use the pointer to it to avoid alignment issues */
    uint32_T localBuffer[(dataLength >> 2) + 1];
    rxData = (uint8_T *)localBuffer;
    /* Retrieve the SCI bus handle from the corresponding handle map */
    sciHandle = getHandle(module, sciMap);
    if ((MW_Handle_Type)NULL != sciHandle) 
    {
        /* Receive the data over SCI */
        *status = (uint8_T)MW_SCI_Receive(sciHandle, rxData, dataLength);
        memcpy(&payloadBufferTx[*peripheralDataSizeResponse], rxData, dataLength);
        *peripheralDataSizeResponse = dataLength;
    }
}
 
/* Trasmit the data over SCI */
void sciTrasmit(uint8_T* payloadBufferRx, uint8_T* payloadBufferTx, uint16_T* peripheralDataSizeResponse, uint8_T* status)
{
    uint32_T module = 0;
    uint32_T dataLength = 0;
    uint8_T index = 0; 
    uint8_T *txData = NULL;
    MW_Handle_Type sciHandle;

    /* Set status as bus error by default */
    *status = (uint8_T)MW_SCI_BUS_ERROR;

    /* Read 4 byte Module of bus from input buffer */
    memcpy(&module, &payloadBufferRx[index], sizeof(uint32_T));
    index += sizeof(uint32_T);
    /* Read 4 byte data length of bus from input buffer */
    memcpy(&dataLength, &payloadBufferRx[index], sizeof(uint32_T));
    index += sizeof(uint32_T);
    /* Define local buffer and use the pointer to it to avoid alignment issues */
    uint32_T localBuffer[(dataLength >> 2) + 1];
    txData = (uint8_T *)localBuffer;
    memcpy(txData, &payloadBufferRx[index], dataLength);
    /* Retrieve the SCI bus handle from the corresponding handle map */
    sciHandle = getHandle(module, sciMap);
    if ((MW_Handle_Type)NULL != sciHandle) 
    {
        /* Trasmit the data over SCI */
        *status = (uint8_T)MW_SCI_Transmit(sciHandle, txData, dataLength);
    }
}


/* Get the status of SCI device */
void getSCIStatus(uint8_T* payloadBufferRx, uint8_T* payloadBufferTx, uint16_T* peripheralDataSizeResponse, uint8_T* status)
{
    uint32_T module = 0;
    uint8_T index = 0; 
    MW_Handle_Type sciHandle;

    /* Set status as bus error by default */
    *status = (uint8_T)MW_SCI_BUS_ERROR;

    /* Read 4 byte Module of bus from input buffer */
    memcpy(&module, &payloadBufferRx[index], sizeof(uint32_T));
    index += sizeof(uint32_T);
    /* Retrieve the SCI bus handle from the corresponding handle map */
    sciHandle = getHandle(module, sciMap);
    if ((MW_Handle_Type)NULL != sciHandle) 
    {
        /* Get the status of SCI device */
         *status = (uint8_T)MW_SCI_GetStatus(sciHandle);
        payloadBufferTx[(*peripheralDataSizeResponse)++] = *status;
    }
}

/* Send break command */
void sciSendBreak(uint8_T* payloadBufferRx, uint8_T* payloadBufferTx, uint16_T* peripheralDataSizeResponse, uint8_T* status)
{
    uint32_T module = 0;
    uint8_T index = 0; 
    MW_Handle_Type sciHandle;

    /* Set status as bus error by default */
    *status = (uint8_T)MW_SCI_BUS_ERROR;

    /* Read 4 byte Module of bus from input buffer */
    memcpy(&module, &payloadBufferRx[index], sizeof(uint32_T));
    index += sizeof(uint32_T);
    /* Retrieve the SCI bus handle from the corresponding handle map */
    sciHandle = getHandle(module, sciMap);
    if ((MW_Handle_Type)NULL != sciHandle) 
    {
        /* Send break command */
        *status = (uint8_T)MW_SCI_SendBreak(sciHandle);
    }
}


/* Release SCI module */
void sciClose(uint8_T* payloadBufferRx, uint8_T* payloadBufferTx, uint16_T* peripheralDataSizeResponse, uint8_T* status)
{
    uint32_T module = 0;
    uint8_T index = 0; 
    MW_Handle_Type sciHandle;
    /* Set status as bus error by default */
    *status = (uint8_T)MW_SCI_BUS_ERROR;
    /* Read 4 byte Module of bus from input buffer */
    memcpy(&module, &payloadBufferRx[index], sizeof(uint32_T));
    index += sizeof(uint32_T);
    /* Retrieve the SCI bus handle from the corresponding handle map */
    sciHandle = getHandle(module, sciMap);
    if ((MW_Handle_Type)NULL != sciHandle) 
    {
        /* Release SCI module */
        MW_SCI_Close(sciHandle);
        *status = (uint8_T)MW_SCI_SUCCESS;
    }
}

#endif
