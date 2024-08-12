/**
 * @file IO_packet.h
 *
 * Header file for IO_packet.c
 *
 * @Copyright 2017-2020 The MathWorks, Inc.
 *
 */

#ifndef IO_PACKET_H_
#define IO_PACKET_H_

#include "rtiostream.h"
#include "IO_requestID.h"
#include "IO_peripheralInclude.h"
#include "IO_utilities.h"
#if CHECKSUM_ENABLE
#include "IO_checksum.h"
#endif
/*
 *  Size representation in uint16_T to enable to scale the protocol to 16bit byte targets. (C2000)
 * */


#ifndef MEMORY_ADJUSTMENT
#define MEMORY_ADJUSTMENT 1
#define rtIOStreamSend8Bits rtIOStreamSend
#define calculate_crc8_j1850_bytePacked calculate_crc8_j1850
#define calculate_crc8_j1850_withInit_bytePacked calculate_crc8_j1850_withInit
#endif

#define RTIOSTREAM_ERROR (-1)

#define HEADER_HOSTTOTARGET	 (0xAA)
#define HEADER_TARGETTOHOST  (0x55)


/*uint8_T is used for data, because every target with an MAU of 16bit or 32 bit uses
 * a 16bit/32bit container to hold a 8 bit data. Its the responsibility of the target author
 * to modify this layer(packeting) to take care of processors with different MAU.
 * I see the only location that will change is the payLoad, where a byte pack would be necessary.
 */

/* "IO_PAYLOADFIELDSIZE" macro should be defined by target author in the peripheralIncludes.h file.
 *  IO_PAYLOADFIELDSIZE is also used to represent the number of bytes the ResponseDataSize field takes.
 *  In case, the user did not define the IO_PAYLOADFIELDSIZE then we fallback to the current shipping solution, ie. "2-1"
 *  Known field sizes for various targets are give below.
 *  S.No. |Target name       |  PayloadFieldSize | ResponseDataFieldSize
 * ------------------------------------------------------------------------------
 *    1   | Arduino(default) |           2       |           1
 *    2   | Raspberry Pi     |           4       |           4
 * 
 * Note: Raspberry Pi IO server supports functionalities like camera snapshot that requires the server to handle huge payloads approx (2-4MB).
 *       Hence Raspberry Pi needs a "4-4" standard
 */
#ifndef IO_PAYLOADFIELDSIZE
    #define IO_PAYLOADFIELDSIZE 2
    #define IO_RESPONSESIZESIZE 1
#else
    #if !(4==IO_PAYLOADFIELDSIZE) && !(2==IO_PAYLOADFIELDSIZE) && !(1==IO_PAYLOADFIELDSIZE)
        #error Unexpected value for IO_PAYLOADFIELDSIZE. The allowed values of IO_PAYLOADFIELDSIZE are {1,2,4}.
    #endif
    #define IO_RESPONSESIZESIZE IO_PAYLOADFIELDSIZE
#endif

#define HEADER_SIZE 	   (sizeof(uint8_T))
#define PAYLOADSIZE_SIZE   IO_PAYLOADFIELDSIZE
#define UNIQUEID_SIZE      (sizeof(uint8_T))
#define REQUESTID_SIZE     (sizeof(uint16_T)*MEMORY_ADJUSTMENT)
#define ISRAWREAD_SIZE     (sizeof(uint8_T))
#define STATUS_SIZE        (sizeof(uint8_T))   //Only used for HOST to Target, Not used for Target to Host

#if 4 == IO_RESPONSESIZESIZE
    #define RESPONSEDATASIZE_SIZE (sizeof(uint32_T)*MEMORY_ADJUSTMENT)
#elif 2 == IO_RESPONSESIZESIZE
    #define RESPONSEDATASIZE_SIZE (sizeof(uint16_T)*MEMORY_ADJUSTMENT)
#elif 1 == IO_RESPONSESIZESIZE
     #define RESPONSEDATASIZE_SIZE (sizeof(uint8_T))
#endif

#define TIMESTAMP_SIZE     (sizeof(float)*MEMORY_ADJUSTMENT)
#define BYTE_ALIGNMENT_FILLER              (sizeof(uint8_T))    // To ensure double alignment of buffer size. IOProtocol overhead = 8 bytes instead of 7.


#if CHECKSUM_ENABLE
#define CHECKSUM_SIZE      (sizeof(uint8_T))
#else
#define CHECKSUM_SIZE      (0)
#endif

#ifndef MAX_PACKET_SIZE
#define MAX_PACKET_SIZE (64)			/* 64 8 bit bytes both in c2000 and other targets. -> corresponding size mentioned in the IOProtocol.m */
#endif

#if (MAX_PACKET_SIZE) > (1 << (8*IO_PAYLOADFIELDSIZE))
    #error MAX_PACKET_SIZE value exceeded the maximum value of packet payloadSize. Please consider reducing the MAX_PACKET_SIZE.
#endif

#define PAYLOAD_SIZE (MAX_PACKET_SIZE - (HEADER_SIZE + PAYLOADSIZE_SIZE + REQUESTID_SIZE + UNIQUEID_SIZE + ISRAWREAD_SIZE + BYTE_ALIGNMENT_FILLER))

#define FUNCTION_NOT_FOUND (1)

#if 4 == IO_PAYLOADFIELDSIZE
    #define payloadSize_T uint32_T
#elif 2 == IO_PAYLOADFIELDSIZE
    #define payloadSize_T uint16_T
#elif 1 == IO_PAYLOADFIELDSIZE
    #define payloadSize_T uint8_T
#endif

#if ((2 == IO_PAYLOADFIELDSIZE) && (1 == IO_RESPONSESIZESIZE))

    typedef struct SimIOpacket_tag{

        uint8_T     uniqueId;
        uint8_T     isRawRead;
        uint8_T     status;
        uint8_T     peripheralDataSizeResponse;
        uint16_T    payloadSize;
        uint16_T    requestId;
        uint16_T    dataPayloadSize;
    #if CHECKSUM_ENABLE
        uint8_T     checksum;
    #endif
        uint8_T     *ptrTxPayLoad;
        uint8_T     *ptrRxPayLoad;

    }simIOpacket;

#else

    typedef struct SimIOpacket_tag{

        uint8_T     uniqueId;
        uint8_T     isRawRead;
        uint8_T     status;
    #if CHECKSUM_ENABLE
        uint8_T     checksum;
    #endif
        uint16_T    requestId;
        payloadSize_T    peripheralDataSizeResponse;
        payloadSize_T    payloadSize;        
        payloadSize_T    dataPayloadSize;
    #if 1 == IO_PAYLOADFIELDSIZE
        /* If ptrTxPayLoad is not on a 4 byte boundary then depending on certain architectures, this could throw an exception.
         * This hard fault could occur in scenarios like custom functions where the ptrTxPayLoad is used as uint32 type.
         * Hence we added 3 explicit bytes for padding when IO_PAYLOADFIELDSIZE is equal to 1.
         */
        uint8_T padByte1;
        uint8_T padByte2;
        uint8_T padByte3;
    #endif
        uint8_T     *ptrTxPayLoad;
        uint8_T     *ptrRxPayLoad;

    }simIOpacket;

#endif

struct packetNode {
    uint8_T	*packet;
    struct  packetNode *pNextPacketNode;
} ;

struct packetLinkedList {
    struct packetNode *header;
    struct packetNode *tail;
};

#ifdef __cplusplus
extern "C" {
#endif
void writeToMATLAB(simIOpacket*);
uint8_T readFromMATLAB(void);
void openConnection(void);
uint8_T calculateChecksum(simIOpacket* packet,uint8_T RxPacket,uint8_T initVal);
void writeToMATLABImmediate(simIOpacket*);
#ifdef __cplusplus
}
#endif
#endif /* IO_PACKET_H_ */
