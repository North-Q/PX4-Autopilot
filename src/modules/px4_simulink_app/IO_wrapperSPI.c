/**
* @file IO_wrapperSPI.c
*
* Contains definition of functions used for SPI communication
*
* @Copyright 2017-2019 The MathWorks, Inc.
*
*/

#include "IO_wrapperSPI.h"
#if IO_STANDARD_SPI
static const uint16_T ZERO = 0;

uint8_T multiByteSPI = 0;

#if 1 == DEBUG_FLAG

#define OPENSPI "OpenSPI::MW_SPI_Open(spiBus %" PRIu32 ",MOSIPin %" PRIu32 ",MISOPin %" PRIu32 ",SCLKPin %" PRIu32 ",CSPin %" PRIu32 ");"
#define SETSPIFORMAT "SetSPIFormat::MW_SPI_SetFormat(spiBus %" PRIu32 ",targetPrecision %" PRIu32 ",spiMode %" PRIu32 ",bitOrder %s);"
#define SETSPIFREQ "SetBusSpeedSPI::MW_SPI_SetBusSpeed(spiBus %" PRIu32 ",frequency %" PRIu32 ");"
#define WRITEREADSPI1 "WriteReadSPI::MW_digitalIO_write(CSPin %" PRIu32 ",1-isCSPinActiveLow %" PRIu16 ");"
#define CLOSESPI "CloseSPI::MW_SPI_Close(spiBus %" PRIu32 ",MOSIPin %" PRIu32 ",MISOPin %" PRIu32 ",SCLKPin %" PRIu32 ",CSPin %" PRIu32 ");"
#define SPIFORLINUX "Enable the SPI peripheral for Linux based target"
#define UNMOUNT "Unmounting the SPI device peripheral"

const char MSG_OPENSPI[] PGMKEYWORD = OPENSPI;
const char MSG_SETSPIFORMAT[] PGMKEYWORD = SETSPIFORMAT;
const char MSG_SETSPIBUSFREQ[] PGMKEYWORD = SETSPIFREQ;
const char MSG_WRITEREADSPI1[] PGMKEYWORD = WRITEREADSPI1;
const char MSG_CLOSESPI[] PGMKEYWORD = CLOSESPI;
const char MSG_SPIFORLINUX[] PGMKEYWORD = SPIFORLINUX;
const char MSG_UNMOUNT[] PGMKEYWORD = UNMOUNT;

#endif

/* The IO_SPI_MODULES_MAX and IO_DIGITALIO_MODULES_MAX macros should be defined in the peripheralIncludes.h file */

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
PeripheralHandleMapType SPIMap[(IO_SPI_MODULES_MAX/8)+((IO_SPI_MODULES_MAX%8)!=0)] = {0};
extern PeripheralHandleMapType digitalIOMap[(IO_DIGITALIO_MODULES_MAX/8)+((IO_DIGITALIO_MODULES_MAX%8)!=0)];

#else
PeripheralHandleMapType SPIMap[IO_SPI_MODULES_MAX] = {NULL};
extern PeripheralHandleMapType digitalIOMap[IO_DIGITALIO_MODULES_MAX]; // The digitalIOMapMap is declared in the DigitalIO wrapper

#endif

/* Function to enable the SPI peripheral for Linux based target*/
void enableSPI(uint16_T* peripheralDataSizeResponse, uint8_T* payloadBufferRx, uint8_T* payloadBufferTx, uint8_T* status)
{
    #if 1 == DEBUG_FLAG
    debugPrint(MSG_SPIFORLINUX);
    #endif
    *status = MW_SPI_SUCCESS;
}

/* Function to set the properties of SPI bus and open the SPI bus*/
void openSPI(uint16_T* peripheralDataSizeResponse, uint8_T* payloadBufferRx, uint8_T* payloadBufferTx, uint8_T* status)
{
    uint16_T index = ZERO;
    uint32_T MOSIPin, MISOPin, clockPin, slaveSelectPin, SPIBus, SPIBusFrequency;
    uint8_T SPITargetPrecision, isSPIDeviceMaster, isCSPinActiveLow;
    MW_SPI_Mode_type SPIMode;
    MW_SPI_FirstBitTransfer_Type bitOrder;
    MW_Handle_Type SPIHandle;
    MW_SPI_Status_Type formatStatus, speedStatus;
    #if 0 == IO_CS_BY_SPI_DRIVER
    MW_Handle_Type digitalIOHandle;
    #endif /* IO_CS_BY_SPI_DRIVER */

    *status = MW_SPI_BUS_ERROR; // Set status as MW_SPI_BUS_ERROR by default
    /* Read SPI device properties from the data received from MATLAB */
    SPIBus = (uint32_T)payloadBufferRx[index++]; // SPI bus number
    MOSIPin = (uint32_T)payloadBufferRx[index++]; // Master Out Slave In pin number
    MISOPin = (uint32_T)payloadBufferRx[index++]; // Master In Slave Out pin number
    clockPin = (uint32_T)payloadBufferRx[index++]; //SPI clock pin number
    slaveSelectPin = (uint32_T)payloadBufferRx[index++]; // SPI slave select pin
    isCSPinActiveLow = payloadBufferRx[index++]; // Defines if the SPI device is an active low or active high enabled device
    isSPIDeviceMaster = payloadBufferRx[index++]; // Defines if the SPI device acts as a master or slave
    SPITargetPrecision = payloadBufferRx[index++]; // Defines the SPI device precision
    SPIMode = (MW_SPI_Mode_type)payloadBufferRx[index++]; // Defines the mode of operation of the SPI device
    bitOrder = (MW_SPI_FirstBitTransfer_Type)payloadBufferRx[index++]; // Defines the order of bit transfer (MSB first or LSB first)
    memcpy(&SPIBusFrequency, &payloadBufferRx[index], sizeof(uint32_T)); // Defines the SPI bit rate
    index = index + sizeof(uint32_T);

    #if 0 == IO_CS_BY_SPI_DRIVER
    digitalIOHandle = MW_digitalIO_open(slaveSelectPin, MW_DIGITALIO_OUTPUT); // Configure the slaveSelectPin for Output
    setHandle(slaveSelectPin, digitalIOHandle, digitalIOMap); // Save the DigitalIO handle in the digitalIOMap
    MW_digitalIO_write(digitalIOHandle, isCSPinActiveLow); //Disable the SPI device
    #endif /* IO_CS_BY_SPI_DRIVER */

    SPIHandle = MW_SPI_Open(SPIBus, MOSIPin, MISOPin, clockPin, slaveSelectPin, isCSPinActiveLow, isSPIDeviceMaster); // Open the SPI bus and save the SPI bus handle to SPIhandle

    #if 1 == DEBUG_FLAG
    /* print debug message */
    debugPrint(MSG_OPENSPI,SPIBus, MOSIPin, MISOPin, clockPin, slaveSelectPin);
    #endif

    #if 0 == IO_CS_BY_SPI_DRIVER
    /* If both the SPI and DigitalIO handles are valid */
    if (((MW_Handle_Type)NULL != SPIHandle) && ((MW_Handle_Type)NULL != digitalIOHandle))
        #else
        /* If SPI handle is valid  */
        if ((MW_Handle_Type)NULL != SPIHandle)
            #endif /* IO_CS_BY_SPI_DRIVER */
        {
            /* set the SPI bus format and speed followed by opening the SPI bus */
            setHandle(SPIBus, SPIHandle, SPIMap); // Save the handle into SPIMap
            formatStatus = MW_SPI_SetFormat(SPIHandle, SPITargetPrecision, SPIMode, bitOrder); // Set SPI Bus format and return the status
            if(MW_SPI_SUCCESS == formatStatus){
                speedStatus = MW_SPI_SetBusSpeed(SPIHandle, SPIBusFrequency); // Set the SPI bus speed and return the status
                *status = speedStatus;
            }
            else {
                *status = formatStatus;
            }
        }
}

/* Function to set the SPI Mode and SPI bit order*/
void setFormatSPI(uint16_T* peripheralDataSizeResponse, uint8_T* payloadBufferRx, uint8_T* payloadBufferTx, uint8_T* status)
{
    uint16_T index = ZERO;
    MW_Handle_Type SPIHandle;
    uint32_T SPIBus;
    uint8_T SPITargetPrecision;
    MW_SPI_Mode_type SPIMode;
    MW_SPI_FirstBitTransfer_Type bitOrder;

    *status = MW_SPI_BUS_ERROR; // Set status as MW_SPI_BUS_ERROR by default
    SPIBus = (uint32_T)payloadBufferRx[index++]; // SPI bus number
    SPITargetPrecision = (uint32_T)payloadBufferRx[index++]; // Defines the SPI device precision
    SPIMode = (uint32_T)payloadBufferRx[index++]; // Defines the mode of operation of the SPI device
    bitOrder = (uint32_T)payloadBufferRx[index++]; // Defines the order of bit transfer (MSB first or LSB first)
    SPIHandle = getHandle(SPIBus, SPIMap); // Get the SPI peripheral(bus) handle from SPIMap
    #if 1 == DEBUG_FLAG
    if (1 == bitOrder)
    {
        /* print debug message */
        debugPrint(MSG_SETSPIFORMAT, SPIBus, SPITargetPrecision, (uint32_T)SPIMode, "lsbfirst");
    }
    else
    {
        /* print debug message */
        debugPrint(MSG_SETSPIFORMAT,(uint32_T)SPIBus,(uint32_T)SPITargetPrecision, (uint32_T)SPIMode, "msbfirst");
    }
    #endif
    if ((MW_Handle_Type)NULL != SPIHandle) {
        *status = MW_SPI_SetFormat(SPIHandle, SPITargetPrecision, SPIMode, bitOrder); // Set SPI Bus format and return the status
    }
}

/* Function to set the SPI bus frequency*/
void setBusSpeedSPI(uint16_T* peripheralDataSizeResponse, uint8_T* payloadBufferRx, uint8_T* payloadBufferTx, uint8_T* status)
{
    uint16_T index = ZERO;
    MW_Handle_Type SPIHandle;
    uint32_T SPIBus,SPIBusFrequency;

    *status = MW_SPI_BUS_ERROR; // Set status as MW_SPI_BUS_ERROR by default
    SPIBus = (uint32_T)payloadBufferRx[index++]; // SPI bus number
    memcpy(&SPIBusFrequency, &payloadBufferRx[index], sizeof(uint32_T)); // Defines the SPI bit rate
    index = index + sizeof(uint32_T);
    SPIHandle = getHandle(SPIBus, SPIMap); // Get the SPI peripheral(bus) handle from SPIMap

    #if 1 == DEBUG_FLAG
    /* print debug message */
    debugPrint(MSG_SETSPIBUSFREQ,SPIBus,SPIBusFrequency);
    #endif

    if ((MW_Handle_Type)NULL != SPIHandle) {
        *status = MW_SPI_SetBusSpeed(SPIHandle, SPIBusFrequency); // Set the SPI bus frequency and return the status
    }
}

void multiByteWriteReadSPI(uint16_T* peripheralDataSizeResponse, uint8_T* payloadBufferRx, uint8_T* payloadBufferTx, uint8_T* status)
{
    multiByteSPI = 1;

    writeReadSPI(peripheralDataSizeResponse, payloadBufferRx, payloadBufferTx, status);

    multiByteSPI = 0;
}

/* Function to write into and read from an SPI device*/
void writeReadSPI(uint16_T* peripheralDataSizeResponse, uint8_T* payloadBufferRx, uint8_T* payloadBufferTx, uint8_T* status)
{
    uint16_T index = ZERO;
    uint32_T SPIBus,dataLength,ctr;
    const uint8_T *wrData;
    uint8_T *rdData;
    uint8_T isCSPinActiveLow;
    MW_Handle_Type SPIHandle;
    uint32_T slaveSelectPin;
    uint8_t multiByteSPILength = 0;

    #if (0 == IO_CS_BY_SPI_DRIVER)
    MW_Handle_Type digitalIOHandle;
    #endif /* IO_CS_BY_SPI_DRIVER */

    /*  Extract the data sent from MATLAB contained in the Receive buffer */
    SPIBus = (uint32_T)payloadBufferRx[index++]; // SPI bus number
    slaveSelectPin = (uint32_T)payloadBufferRx[index++]; // slave select pin number
    isCSPinActiveLow = (uint8_T)payloadBufferRx[index++]; // Defines if the SPI device is an active low or active high enabled device
    memcpy(&dataLength, &payloadBufferRx[index], sizeof(uint32_T)); // Data length in bytes
    index = index + sizeof(uint32_T);

    /* Defining local buffers and using pointers for local buffers to avoid alignment issues */
    uint32_T localBufferWriteData[(dataLength >> 2) + 1];
    uint32_T localBufferReadData[(dataLength >> 2) + 1];
    wrData = (uint8_T *)localBufferWriteData; // Pointer to the local buffer containing data to be written
    memcpy(wrData, &payloadBufferRx[index], dataLength); // Copy data to be written to local buffer

    rdData = (uint8_T *)localBufferReadData;  // Pointer to the local buffer to hold read data

    if(multiByteSPI)   {
        multiByteSPILength = wrData[dataLength-1];
    }

    *status = MW_SPI_BUS_ERROR; // Set status as MW_SPI_BUS_ERROR by default

    /* Retrieve the SPI bus handle */
    SPIHandle = getHandle(SPIBus, SPIMap);
    #if 0 == IO_CS_BY_SPI_DRIVER
    /* Retrieve the DigitalIO handle corresponding to slave select pin from the respective handle map*/
    digitalIOHandle = getHandle(slaveSelectPin, digitalIOMap);
    /* If both the DigitalIO and SPI handle are valid, then write data to SPI */
    if ((MW_Handle_Type)NULL != SPIHandle && (MW_Handle_Type)NULL != digitalIOHandle)
        #else
        /* If SPI handle are valid, then write data to SPI without handling the CS pin here */
        if ((MW_Handle_Type)NULL != SPIHandle)
            #endif /* IO_CS_BY_SPI_DRIVER */
        {
            #if 0 == IO_CS_BY_SPI_DRIVER
            MW_digitalIO_write(digitalIOHandle,1-isCSPinActiveLow); // Enable the SPI device before writing to device
            #else
            /* The following mentions to the driver whether CS Pin is Active Low or not so that driver can set it accordingly
            In the previous API, we are using the Active Low status to drive the pin and hence the manipulation (1-CSPinActiveLow)
            When Driver is managing CS, this function is used to set the current CS Pin. */
            *status = MW_SPI_SetSlaveSelect(SPIHandle, slaveSelectPin, isCSPinActiveLow);
            #endif /* IO_CS_BY_SPI_DRIVER */

            #if 1 == DEBUG_FLAG
            /* print debug message */
            debugPrint(MSG_WRITEREADSPI1, slaveSelectPin, ((uint8_T)1-(uint8_T)isCSPinActiveLow));
            #endif

            if (multiByteSPILength)    {
                uint8_T k=0, loop=0;
                // Read the data for every 'multiByteSPILength'
                for(loop=0;loop<(dataLength-1)/multiByteSPILength;loop++)    {
                    *status = MW_SPI_MasterWriteRead_8bits(SPIHandle, &wrData[k], &rdData[k], multiByteSPILength); // Write the data to SPI device and return status
                    k=k+multiByteSPILength;
                }
                // When multiByteSPILength is set, an additional byte is appended to the read address, and the response value of that extra byte is configured to zero
                rdData[k] = 0;
            }
            else	{
                *status = MW_SPI_MasterWriteRead_8bits(SPIHandle, wrData, rdData, dataLength); // Write the data to SPI device and return status
            }

            #if 0 == IO_CS_BY_SPI_DRIVER
            MW_digitalIO_write(digitalIOHandle,isCSPinActiveLow); // Disable the SPI device after writing to device
            #endif /* IO_CS_BY_SPI_DRIVER */
            if (MW_SPI_SUCCESS == *status) {
                /* Store the data that is read from SPI device in the transmit buffer */
                memcpy(&payloadBufferTx[*peripheralDataSizeResponse], rdData, dataLength);
                *peripheralDataSizeResponse = dataLength;
            }
        }
}

/* Function to close the SPI bus */
void closeSPI(uint16_T* peripheralDataSizeResponse, uint8_T* payloadBufferRx, uint8_T* payloadBufferTx, uint8_T* status)
{
    uint16_T index = ZERO;
    uint32_T MOSIPin, MISOPin, clockPin, slaveSelectPin, SPIBus;
    MW_Handle_Type SPIHandle;
    #if 0 == IO_CS_BY_SPI_DRIVER
    MW_Handle_Type digitalIOHandle;
    #endif /* IO_CS_BY_SPI_DRIVER */

    *status = MW_SPI_BUS_ERROR; // Set status as MW_SPI_BUS_ERROR by default
    /* Read the SPI bus properties from the Receive buffer */
    SPIBus = (uint32_T)payloadBufferRx[index++];
    MOSIPin = (uint32_T)payloadBufferRx[index++];
    MISOPin = (uint32_T)payloadBufferRx[index++];
    clockPin = (uint32_T)payloadBufferRx[index++];
    slaveSelectPin = (uint32_T)payloadBufferRx[index++];

    /* Retrieve the SPI bus handle from the respective handle map*/
    SPIHandle = getHandle(SPIBus, SPIMap);
    #if 0 == IO_CS_BY_SPI_DRIVER
    /* Retrieve the DigitalIO handle corresponding to slave select pin from the respective handle map*/
    digitalIOHandle = getHandle(slaveSelectPin, digitalIOMap);
    /* If SPI and digital pin CSP handle is vaild, then close the SPI bus and return MW_SPI_SUCCESS. If either of the above handles are invalid, return MW_SPI_BUS_ERROR*/
    if ((MW_Handle_Type)NULL != SPIHandle && (MW_Handle_Type)NULL != digitalIOHandle)
        #else
        /* If SPI handle is vaild, then close the SPI bus and return MW_SPI_SUCCESS. If handles is invalid, return MW_SPI_BUS_ERROR*/
        if ((MW_Handle_Type)NULL != SPIHandle)
            #endif /* IO_CS_BY_SPI_DRIVER */
        {
            MW_SPI_Close(SPIHandle, MOSIPin, MISOPin, clockPin, slaveSelectPin); // Close SPI bus

            #if 1 == DEBUG_FLAG
            /* print debug message */
            debugPrint(MSG_CLOSESPI, SPIBus, MOSIPin, MISOPin, clockPin, slaveSelectPin);
            #endif
            #if 0 == IO_CS_BY_SPI_DRIVER
            MW_digitalIO_close(digitalIOHandle); // Close the Slave Select pin
            setHandle(slaveSelectPin,(MW_Handle_Type)NULL,digitalIOMap); // Make the corresponding DigitalIO handle to NULL
            #endif /* IO_CS_BY_SPI_DRIVER */
            setHandle(SPIBus,(MW_Handle_Type)NULL,SPIMap); // Make the corresponding SPI handle to NULL
            *status = MW_SPI_SUCCESS; // Always passes because SVD doesn't return a status
        }
}

/* Function to disable or unmount the SPI device peripheral*/
void disableSPI(uint16_T* peripheralDataSizeResponse, uint8_T* payloadBufferRx, uint8_T* payloadBufferTx, uint8_T* status)
{
    #if 1 == DEBUG_FLAG
    /* print debug message */
    debugPrint(MSG_UNMOUNT);
    #endif
    *status = MW_SPI_SUCCESS;
}

#endif
