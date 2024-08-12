/**
* @file IO_memOperations.h
*
* Contains declarations of memory read and memory write functions.
*
* @Copyright 2023 The MathWorks, Inc.
*
*/
#ifndef IO_MEMOPERATIONS_H
#define IO_MEMOPERATIONS_H

#include "rtwtypes.h"
#include "IO_debug.h"
#include "IO_packet.h"

void memoryWrite(uint8_T* payloadBufferRx);
void memoryRead(uint8_T* payloadBufferRx, uint8_T* payloadBufferTx, uint16_T* peripheralDataSizeResponse);
void memoryBitWrite(uint8_T* payloadBufferRx);
void getBusWidth(uint8_T *payloadBufferTx, payloadSize_T *peripheralDataSizeResponse, uint8_T *status);

#ifndef __AVR_ARCH__
uint64_T packBytesInto64BitInteger(uint8_T*);
void unpack64BitstoByte(uint64_T, uint8_T*);
#endif

#if IO_STANDARD_MEMORY_OPERATIONS_PROTECTION_AVAILABLE
//Disbale memory protection
extern void disableMemoryProtectionFunction(uint8_T* payloadBufferRx);
//Enable memory protection
extern void enableMemoryProtectionFunction(uint8_T* payloadBufferRx);
#endif

#endif
