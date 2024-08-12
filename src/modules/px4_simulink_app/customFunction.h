/* Copyright 2020-2023 The MathWorks, Inc. */
#ifndef CUSTOMFUNCTION_H
#define CUSTOMFUNCTION_H

// Uncomment the line below to enable debug option
// #define DEBUG_FLAG 1

#include "IO_include.h"
#include "IO_peripheralInclude.h"
#include "MW_uORB_Read.h"
#include "MW_uORB_Write.h"
#include "MW_ParameterRead.h"
#include "PeripheralToHandle.h"
#include "MW_PX4_PWM.h"

#if defined(MW_PX4_NUTTX_BUILD)
#include "MW_PX4_SCI.h"
#endif // #if defined(MW_PX4_NUTTX_BUILD)

#include <uORB/uORB.h>
#include <drivers/drv_hrt.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum customFunctionRequestID {
    /*===========================================
     * Custom Function Request ID
     * Request ID should be between 0x100 - 0xFFF
     *==========================================*/
    // uORB
    UORBREADINIT = 0x100,
    UORBWRITEINIT = 0x101,
    UORBREAD = 0x102,
    UORBWRITE = 0x103,
    UORBREADTERMINATE = 0x104,
    UORBWRITETERMINATE = 0x105,
    // Read Param
    PARAMINIT = 0x106,
    PARAMSTEP = 0x107,
    // Serial
    SCIGETAVAILABLEBYTES = 0x108,
    // PWM
    PWM_OPEN = 0x109,
    PWM_ARM = 0x10A,
    PWM_DISARM = 0x10B,
    PWM_FORCEFS = 0x10C,
    PWM_SETSERVO = 0x10D,
    PWM_RESETSERVO = 0x10E,
    PWM_CLOSE = 0x10F,
    // CAN
    CAN_OPEN = 0x110,
    CAN_TRANSMIT = 0x111,
    CAN_ASSIGNBUFFERID = 0x112,
    CAN_RECEIVEBYID = 0x113,
    CAN_RECEIVEMESSAGE = 0x114,
    CAN_CLOSE = 0x115,
    // HRT ABS TIME
    GET_HRTABSTIME = 0x116

} requestIDs;

void customFunctionHookInit();
void customFunctionHook(uint16_T cmdID,
                        uint8_T* payloadBufferRx,
                        uint8_T* payloadBufferTx,
                        uint16_T* peripheralDataSizeResponse);

#ifdef __cplusplus
}
#endif

#endif
