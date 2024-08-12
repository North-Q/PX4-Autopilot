/* Copyright 2020-2021 The MathWorks, Inc. */
#ifndef __MW_CAN_H__
#define __MW_CAN_H__

#include "MW_SVD.h"

typedef int32_T MW_CAN_Status_T;

#ifdef __cplusplus
extern "C" {
#endif
    
    typedef struct
    {
        uint8_T ProtocolMode;
        uint8_T Extended;
        uint8_T Length;
        uint8_T Remote;
        uint8_T Error;
        uint8_T BRS;
        uint8_T ESI;
        uint8_T DLC;
        uint32_T ID;
        uint32_T Reserved;
        real_T Timestamp;
        uint8_T Data[64];
    }MW_CANFD_MSG_T;

    extern MW_Handle_Type MW_CAN_Open(uint32_T CANModule);
    extern MW_CAN_Status_T MW_CAN_ReceiveMessages_By_ID(MW_Handle_Type CANHandle, uint32_T Id, uint8_T * Data, uint8_T Length, uint8_T *Remote, uint8_T Format, uint32_T rxBufferIndex);
    extern MW_CAN_Status_T MW_CAN_ReceiveMessages(MW_Handle_Type CANHandle, uint32_T *Id, uint8_T *Data, uint8_T *Length, uint8_T *Remote, uint8_T *Format, uint32_T rxBufferIndex);
    extern MW_CAN_Status_T MW_CAN_TransmitMessage(MW_Handle_Type CANHandle, uint32_T Id, uint8_T * Data, uint8_T Length, uint8_T Remote, uint8_T Format);
    extern uint8_T MW_CAN_ConfigureRxFilter(MW_Handle_Type CANHandle, uint32_T id, uint32_T mask, uint32_T format, int32_T handle);
    extern MW_CAN_Status_T MW_CAN_Close(MW_Handle_Type CANHandle);
    extern MW_CAN_Status_T MW_CANFD_ReceiveMessages(MW_Handle_Type CANHandle, MW_CANFD_MSG_T *MW_ReceiveMsg, uint32_T rxBufferIndex);
    extern MW_CAN_Status_T MW_CANFD_TransmitMessage(MW_Handle_Type CANHandle, uint32_T Id, const uint8_T * Data, uint8_T Length, uint8_T Remote, uint8_T Format, uint8_T CANFrameFormat,uint8_T isBRSEnable);
    
#ifdef __cplusplus
}

#endif

#endif /* __MW_CAN_H__ */

