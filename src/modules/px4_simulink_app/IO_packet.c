/**
* @file IO_packet.c
*
* Contains declaration of functions used for receiving and sending the IO packets over transport layer.
*
* @Copyright 2017-2023 The MathWorks, Inc.
*
*/

#include "IO_packet.h"

extern void disableGlobalInterrupt(void);
extern void enableGlobalInterrupt(void);

void blockingRead(void *dst, size_t sizeToRead, size_t *sizeRead);
// This is for checking if the header of any packet has arrived without blocking the streaming mode operation
void nonblockingRead(void *dst, size_t sizeToRead, size_t *sizeRead);

simIOpacket RcvdPacket;
static uint8_T RxPacketHeader;
const uint8_T TxPacketHeader = HEADER_TARGETTOHOST;

// Buffer to read the complete packet data. Used by transports like BLE where
// communication happens packet-wise
#ifdef MW_IOSERVER_FIXED_FRAME_COMMUNICATION
uint8_T rcvdBuffer[MAX_PACKET_SIZE] = {0};
#endif

#if CHECKSUM_ENABLE
uint8_T crcMallocFailedStreamCon = 0;
#endif

void openConnection(void)
{
    rtIOStreamOpen(0,0);
}

uint8_T readFromMATLAB(void)
{
    size_t payloadSize=0;
    size_t sizeRecvd=0;

    #if CHECKSUM_ENABLE
    uint8_T payloadSizeChecksum = 0;
    uint8_T payloadSizeCalculatedChecksum = 0;
    #endif
    
    #if (1 == IO_PAYLOADFIELDSIZE || 2 == IO_PAYLOADFIELDSIZE)
    uint8_T temp[2]={0,0}; // min size is 2 because of request ID size
    #else
    uint8_T temp[IO_PAYLOADFIELDSIZE]={0};
    #endif
    payloadSize_T packBytesIntoIntegerData = 0;

    #if !defined(MW_IOSERVER_FIXED_FRAME_COMMUNICATION)  // Every byte is read from the comm line like Serial
    // Only for Soft Real Time implementation
    nonblockingRead((uint8_T*)&RxPacketHeader, (size_t)HEADER_SIZE, &sizeRecvd);
    if((sizeRecvd == 1) && (RxPacketHeader == HEADER_HOSTTOTARGET))
    {
        blockingRead(temp, (size_t)(PAYLOADSIZE_SIZE), &sizeRecvd);
        #if 1 == IO_PAYLOADFIELDSIZE
        memcpy((uint8_T*)&RcvdPacket.payloadSize,temp,sizeof(payloadSize_T));
        packBytesIntoIntegerData = (payloadSize_T)temp;
        #elif 2 == IO_PAYLOADFIELDSIZE
        packBytesIntoIntegerData = (payloadSize_T)packBytesInto16BitInteger(temp);
        memcpy((uint8_T*)&RcvdPacket.payloadSize,&packBytesIntoIntegerData,sizeof(payloadSize_T));
        #elif 4 == IO_PAYLOADFIELDSIZE
        packBytesIntoIntegerData = (payloadSize_T)packBytesInto32BitInteger(temp);
        memcpy((uint8_T*)&RcvdPacket.payloadSize,&packBytesIntoIntegerData,sizeof(payloadSize_T));
        #endif
        //If checksum is enabled, validate payload length using payload checksum sent after the payload bytes
        #if CHECKSUM_ENABLE
        blockingRead((uint8_T*)&payloadSizeChecksum,(size_t)CHECKSUM_SIZE, &sizeRecvd);
        payloadSizeCalculatedChecksum = calculate_crc8_j1850((const uint8_T *)&RcvdPacket.payloadSize,(uint16_T)0,(uint16_T)IO_PAYLOADFIELDSIZE);
        if (payloadSizeChecksum!=payloadSizeCalculatedChecksum)
        {
            return 0;
        }
        // Exclude payload checksum from the payloadsize
        RcvdPacket.payloadSize =  RcvdPacket.payloadSize-(size_t)CHECKSUM_SIZE;
        #endif
        payloadSize = RcvdPacket.payloadSize;
        
        blockingRead((uint8_T*)&RcvdPacket.uniqueId, (size_t)UNIQUEID_SIZE, &sizeRecvd);
        payloadSize-=(uint8_T)UNIQUEID_SIZE;        //uniqueId - read complete

        blockingRead(temp, (size_t)(REQUESTID_SIZE), &sizeRecvd);
        packBytesIntoIntegerData = (uint16_T)packBytesInto16BitInteger(temp);
        memcpy((uint8_T*)&RcvdPacket.requestId,&packBytesIntoIntegerData,sizeof(uint16_T));
        payloadSize-=(uint8_T)REQUESTID_SIZE;

        blockingRead((uint8_T*)&RcvdPacket.isRawRead, (size_t)ISRAWREAD_SIZE, &sizeRecvd);
        payloadSize-=(uint8_T)ISRAWREAD_SIZE;       //isRawRead - read complete

        if(RcvdPacket.isRawRead)
        {
            blockingRead((uint8_T*)&RcvdPacket.peripheralDataSizeResponse, (size_t)RESPONSEDATASIZE_SIZE, &sizeRecvd); 
            payloadSize-=(uint8_T)RESPONSEDATASIZE_SIZE;  // peripheralDataSizeResponse - read complete
        }
        else
        {
            RcvdPacket.peripheralDataSizeResponse = 0;
        }

        RcvdPacket.dataPayloadSize = payloadSize;
        blockingRead((uint8_T*)RcvdPacket.ptrRxPayLoad, (size_t)(RcvdPacket.dataPayloadSize), &sizeRecvd);
        /*
        * If received header for the packet
        * then read the rest of the content from the
        * packet, else wait until a header is received
        * if data is dropped on the serial line.  We
        * continue to search for the header.
        * If header is present in the payLoad then checksum
        * is a validation mechanism for correct data.
        * The protocol assumes that there is no data dropped
        * during communication.
        * Host is aware of the size of the target buffer, so the
        * data transfer from host is regulated
        * */
        #if CHECKSUM_ENABLE
        blockingRead((uint8_T*)&RcvdPacket.checksum, (size_t)CHECKSUM_SIZE, &sizeRecvd);
        // For a Rx packet, payload length checksum is appended with the packet. In the packet checksum 
        // calculation, this value is also considered.
        uint8_T initVal =  calculate_crc8_j1850((const uint8_T *)&payloadSizeChecksum,(uint16_T)0,(uint16_T)CHECKSUM_SIZE);
        // This value will be passed as initval to this function
        if (RcvdPacket.checksum == calculateChecksum((simIOpacket*) &RcvdPacket,1,initVal))
        {
            return 1;
        }
        else
        {
            return 0;
        }
        #else
        return 1;
        #endif
    }

    #else  // Used by transports like BLE that only support packet-wise data transfer
    // When BLE transport is used, communication happens packet-wise through
    // characteristics. Hence, whole packet is read at a time and parsed accordingly.
    nonblockingRead(rcvdBuffer, MAX_PACKET_SIZE, &sizeRecvd);
    if (sizeRecvd) {
        uint8_T* buf = rcvdBuffer;
        if (buf[0] == HEADER_HOSTTOTARGET) {
            buf++;
            #if 1 == IO_PAYLOADFIELDSIZE
            memcpy((uint8_T*)&RcvdPacket.payloadSize,buf,sizeof(payloadSize_T));
            #elif 2 == IO_PAYLOADFIELDSIZE
            packBytesIntoIntegerData = (payloadSize_T)packBytesInto16BitInteger(buf);
            memcpy((uint8_T*)&RcvdPacket.payloadSize,&packBytesIntoIntegerData,sizeof(payloadSize_T));
            #elif 4 == IO_PAYLOADFIELDSIZE
            packBytesIntoIntegerData = (payloadSize_T)packBytesInto32BitInteger(buf);
            memcpy((uint8_T*)&RcvdPacket.payloadSize,&packBytesIntoIntegerData,sizeof(payloadSize_T));
            #endif
            buf+=sizeof(payloadSize_T);
            //If checksum is enabled, validate payload length using payload checksum sent after the payload bytes
            #if CHECKSUM_ENABLE
            memcpy((uint8_T*)&payloadSizeChecksum,buf,(size_t)CHECKSUM_SIZE);
            buf+=(size_t)(CHECKSUM_SIZE);
            payloadSizeCalculatedChecksum = calculate_crc8_j1850((const uint8_T *)&RcvdPacket.payloadSize,(uint16_T)0,(uint16_T)IO_PAYLOADFIELDSIZE);
            if (payloadSizeChecksum!=payloadSizeCalculatedChecksum)
            {
                return 0;
            }
            // Exclude payload checksum from the payloadsize
            RcvdPacket.payloadSize =  RcvdPacket.payloadSize-(size_t)CHECKSUM_SIZE;
            #endif
            payloadSize = RcvdPacket.payloadSize;

            memcpy((uint8_T*)&RcvdPacket.uniqueId, buf, (size_t)UNIQUEID_SIZE);
            buf += UNIQUEID_SIZE;
            payloadSize -= (uint8_T)UNIQUEID_SIZE;

            packBytesIntoIntegerData = (uint16_T)packBytesInto16BitInteger(buf);
            memcpy((uint8_T*)&RcvdPacket.requestId, &packBytesIntoIntegerData, (size_t)REQUESTID_SIZE);
            buf += REQUESTID_SIZE;
            payloadSize -= (uint8_T)REQUESTID_SIZE;

            memcpy((uint8_T*)&RcvdPacket.isRawRead, buf, (size_t)ISRAWREAD_SIZE);
            buf += ISRAWREAD_SIZE;
            payloadSize -= (uint8_T)ISRAWREAD_SIZE;

            if(RcvdPacket.isRawRead)
            {
                memcpy((uint8_T*)&RcvdPacket.peripheralDataSizeResponse, buf, (size_t)RESPONSEDATASIZE_SIZE);
                buf += RESPONSEDATASIZE_SIZE;
                payloadSize -= (uint8_T)RESPONSEDATASIZE_SIZE;
            }
            else
            {
                RcvdPacket.peripheralDataSizeResponse = 0;
            }

            RcvdPacket.dataPayloadSize = payloadSize;
            memcpy((uint8_T*)RcvdPacket.ptrRxPayLoad, buf, (size_t)(RcvdPacket.dataPayloadSize));
            buf += RcvdPacket.dataPayloadSize;
            #if CHECKSUM_ENABLE
            memcpy((uint8_T*)&RcvdPacket.checksum, buf, (size_t)CHECKSUM_SIZE);
            // For a Rx packet, payload length checksum is appended with the packet. In the packet checksum 
            // calculation, this value is also considered.
            uint8_T initVal =  calculate_crc8_j1850((const uint8_T *)&payloadSizeChecksum,(uint16_T)0,(uint16_T)CHECKSUM_SIZE);
            // This value will be passed as initval to this function
            if (RcvdPacket.checksum == calculateChecksum((simIOpacket*) &RcvdPacket,1,initVal))
            {
                return 1;
            }
            else
            {
                return 0;
            }
            #else
            return 1;
            #endif
        }
    }
    #endif
    return 0;

}

void blockingRead(void *dst, size_t sizeToRead, size_t *sizeRead)
{
    size_t size=0;
    uint8_T* dst_temp = (uint8_T*)dst;
    int8_T status = 0;
    *sizeRead = 0;
    while(*sizeRead < sizeToRead)
    {
        status = rtIOStreamRecv((int)0, &dst_temp[*sizeRead], (size_t)(sizeToRead-*sizeRead), &size);
        *sizeRead+=size;
        if(status == RTIOSTREAM_ERROR)
        {
            rtIOStreamOpen(0,0);
        }
    }

    //<C2000 change here> For C2000 byte pack the 2 eight bit header into a 16 bit header

}

/* If there is no data in the serial buffer or the characteristic (in BLE),
* the loop does not wait till the data comes. It rather goes back and resumes normal operation.
* Required only for Soft Real Time implementation.
*/

/* This function is exactly same as blockingRead except this does not wait in a while loop until desired bytes are collected
*/
void nonblockingRead(void *dst, size_t sizeToRead, size_t *sizeRead)
{
    rtIOStreamRecv((int)0, &((uint8_T*)dst)[*sizeRead], (size_t)sizeToRead, sizeRead);
}

void writeToMATLABImmediate(simIOpacket *packet)
{
    size_t sizeSent=0;
    /* Send the packet immediately */
    #if !defined(BULK_WRITE) && !defined(MW_IOSERVER_FIXED_FRAME_COMMUNICATION)
    rtIOStreamSend8Bits((int)0, &TxPacketHeader, (size_t)HEADER_SIZE, &sizeSent);
    rtIOStreamSend((int)0, &packet->payloadSize, (sizeof(payloadSize_T)*MEMORY_ADJUSTMENT), &sizeSent);
    rtIOStreamSend8Bits((int)0, &packet->uniqueId, (size_t)UNIQUEID_SIZE, &sizeSent);
    rtIOStreamSend((int)0, &packet->requestId, (sizeof(uint16_T)*MEMORY_ADJUSTMENT), &sizeSent);
    rtIOStreamSend8Bits((int)0, &packet->isRawRead, (size_t)ISRAWREAD_SIZE, &sizeSent);
    rtIOStreamSend8Bits((int)0, &packet->status, (size_t)STATUS_SIZE, &sizeSent);
    
    while(packet->dataPayloadSize > (payloadSize_T)PAYLOAD_SIZE)
    {
        rtIOStreamSend8Bits((int)0, packet->ptrTxPayLoad, (size_t)PAYLOAD_SIZE, &sizeSent);
        packet->ptrTxPayLoad = packet->ptrTxPayLoad + (payloadSize_T)PAYLOAD_SIZE;
        packet->dataPayloadSize = packet->dataPayloadSize - (payloadSize_T)PAYLOAD_SIZE;
    }
    rtIOStreamSend8Bits((int)0, packet->ptrTxPayLoad, (size_t)packet->dataPayloadSize, &sizeSent);
    #if CHECKSUM_ENABLE
    rtIOStreamSend8Bits((int)0, &packet->checksum, (size_t)CHECKSUM_SIZE, &sizeSent);
    #endif

    #else

    size_t bytesRead=0;
    static uint8_T msg[MAX_PACKET_SIZE];

    memcpy(&msg[bytesRead], &TxPacketHeader,(size_t)HEADER_SIZE);
    bytesRead+=(size_t)HEADER_SIZE;

    memcpy(&msg[bytesRead], &packet->payloadSize, sizeof(payloadSize_T));
    bytesRead+=sizeof(payloadSize_T);

    memcpy(&msg[bytesRead], &packet->uniqueId, (size_t)UNIQUEID_SIZE);
    bytesRead+=(size_t)UNIQUEID_SIZE;

    memcpy(&msg[bytesRead], &packet->requestId, sizeof(uint16_T));
    bytesRead+=sizeof(uint16_T);

    memcpy(&msg[bytesRead], &packet->isRawRead, (size_t)ISRAWREAD_SIZE);
    bytesRead+=(size_t)ISRAWREAD_SIZE;

    memcpy(&msg[bytesRead], &packet->status, (size_t)STATUS_SIZE);
    bytesRead+=(size_t)STATUS_SIZE;

    memcpy(&msg[bytesRead], packet->ptrTxPayLoad, (size_t)packet->dataPayloadSize);
    bytesRead+=(size_t)packet->dataPayloadSize;

    #if CHECKSUM_ENABLE
    memcpy(&msg[bytesRead], &packet->checksum, (size_t)CHECKSUM_SIZE);
    bytesRead+=(size_t)CHECKSUM_SIZE;
    #endif
    rtIOStreamSend((int)0, msg, (size_t)bytesRead, &sizeSent);

    #endif

}

void writeToMATLAB(simIOpacket* packet)
{
    packet->payloadSize = packet->dataPayloadSize + UNIQUEID_SIZE + REQUESTID_SIZE + ISRAWREAD_SIZE + STATUS_SIZE;

    #if CHECKSUM_ENABLE
    /* Calculate the checksum */
    packet->checksum = calculateChecksum((simIOpacket*) packet,0,0xFF);
    #endif
    writeToMATLABImmediate((simIOpacket*) packet);
    // for targets with MAU other than 8bit, code change is required here
}

#if CHECKSUM_ENABLE
uint8_T calculateChecksum(simIOpacket* packet,uint8_T RxPacket,uint8_T initVal)
{
    uint8_T checksumCalculated = 0;
    checksumCalculated = calculate_crc8_j1850_withInit((const uint8_T *)&packet->uniqueId,initVal,(uint16_T)0,(uint16_T)UNIQUEID_SIZE); /* Calculate uniqueID checksum */
    checksumCalculated = calculate_crc8_j1850_withInit_bytePacked((const uint8_T *)&packet->requestId,(uint8_T)checksumCalculated,(uint16_T)0,(uint16_T)REQUESTID_SIZE); /* Calculate request ID checksum */
    checksumCalculated = calculate_crc8_j1850_withInit((const uint8_T *)&packet->isRawRead,(uint8_T)checksumCalculated,(uint16_T)0,(uint16_T)ISRAWREAD_SIZE); /* Calculate is raw read checksum */
    if (RxPacket)
    {
        if(RxPacket && (packet->isRawRead & 0x01))
        {
            /* Calculate the checksum for response size which is only present in received isRawRead packets */
            checksumCalculated = calculate_crc8_j1850_withInit((const uint8_T *)&packet->peripheralDataSizeResponse,(uint8_T)checksumCalculated,(uint16_T)0,(uint16_T)(RESPONSEDATASIZE_SIZE)); /* Calculate response data size checksum */
        }
        checksumCalculated = calculate_crc8_j1850_withInit((const uint8_T *)packet->ptrRxPayLoad,(uint8_T)checksumCalculated,(uint16_T)0,(uint16_T)(packet->dataPayloadSize)); /* Calculate payload checksum */
    }
    else
    {
        /* Calculate the checksum for status which is only present in Tx packets */
        checksumCalculated = calculate_crc8_j1850_withInit((const uint8_T *)&packet->status,(uint8_T)checksumCalculated,(uint16_T)0,(uint16_T)(STATUS_SIZE)); /* Calculate status checksum */
        checksumCalculated = calculate_crc8_j1850_withInit((const uint8_T *)packet->ptrTxPayLoad,(uint8_T)checksumCalculated,(uint16_T)0,(uint16_T)(packet->dataPayloadSize)); /* Calculate payload checksum */
    }

    return checksumCalculated;
}

#endif
