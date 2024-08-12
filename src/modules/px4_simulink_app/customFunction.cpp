/* Copyright 2020-2024 The MathWorks, Inc. */
#include "customFunction.h"
#include "uORBMetaData.h"
#include "IO_packet.h"
extern simIOpacket RcvdPacket;
#if defined(MW_PX4_NUTTX_BUILD)
extern PeripheralHandleMapType sciMap[];
#ifndef MW_PX4_CAN_DISABLE
#include "MW_PX4_CAN.h"
#endif // #ifndef MW_PX4_CAN_DISABLE
#endif


/* Init Custom peripherals */
void customFunctionHookInit() {}

/* Hook to add the custom peripherals - implemented for uORB Read and Write */
void customFunctionHook(uint16_T requestID,
                        uint8_T* payloadBufferRx,
                        uint8_T* payloadBufferTx,
                        uint16_T* peripheralDataSizeResponse) {
    uint8_T index = 0;
    // for uORb blocks
    orb_metadata_t* orbData;
    pollfd_t eventStructObj;
    uint8_T uORBID = 0;
    orb_advert_t orbAdvertObj;
    // for Read Parameter block
    MW_Param_Handle paramHandle;
    // for PWM block
    boolean_T isMain;
    orb_advert_t armAdvertiseObj;
    orb_advert_t actuatorAdvertiseObj;
    unsigned int servoCount = 0;
    int channelMask;
    uint16_T pwm_value[8];
    // for CAN Block
#if defined(MW_PX4_NUTTX_BUILD) && \
    !defined(MW_PX4_CAN_DISABLE) /* CAN is not supported for PX4 Host Target */
    uint8_T CANModule;
    uint32_T id;
    uint8_T idType;
    uint8_T length;
    uint8_T remote;
    MW_CAN_Status_T errorStatus;

#endif

    switch (requestID) {
        /*===========================================
         * uORB
         *==========================================*/
    case UORBREADINIT: {
        index = 0;
        /* Read 1 byte uORB ID of bus from input buffer */
        memcpy(&uORBID, &payloadBufferRx[index], sizeof(uint8_T));
        /*Get uORB Meta data*/
        orbData = getorbData(uORBID);
        /*Initialize uORB read*/
        uORB_read_initialize(orbData, &eventStructObj);
        memcpy(&payloadBufferTx[0], &orbData, sizeof(orbData));
        memcpy(&payloadBufferTx[sizeof(orbData)], &eventStructObj, sizeof(pollfd_t));
        *peripheralDataSizeResponse = sizeof(orbData) + sizeof(pollfd_t);
        break;
    }
    case UORBWRITEINIT: {
        index = 0;
        int queueLen = 0;
        /* Read 1 byte uORB ID of bus from input buffer */
        memcpy(&uORBID, &payloadBufferRx[index], sizeof(uint8_T));
        index += sizeof(uint8_T);
        /* Read 4 byte Queue length of bus from input buffer */
        memcpy(&queueLen, &payloadBufferRx[index], sizeof(int));
        index += sizeof(int);
        /*Get uORB Meta data*/
        orbData = getorbData(uORBID);
        /*Initialize uORB write*/
        uORB_write_initialize(orbData, &orbAdvertObj, &payloadBufferRx[index], queueLen);
        memcpy(&payloadBufferTx[0], &orbData, sizeof(orbData));
        memcpy(&payloadBufferTx[sizeof(orbData)], &orbAdvertObj, sizeof(orb_advert_t));
        *peripheralDataSizeResponse = sizeof(orbData) + sizeof(orb_advert_t);
        break;
    }
    case UORBREAD: {
        index = 0;
        uint8_T tempPayloadBufferTx[PAYLOAD_SIZE];
        boolean_T blockingMode = 0;
        double blockingTimeout = 0;
        boolean_T updated = 0;
        /* Read 8 byte orbData of bus from input buffer */
        memcpy(&orbData, &payloadBufferRx[index], sizeof(orbData));
        index += sizeof(orbData);
        /* Read 24 byte orbData of bus from input buffer */
        memcpy(&eventStructObj, &payloadBufferRx[index], sizeof(pollfd_t));
        index += sizeof(pollfd_t);
        /* Read 1 byte blocking mode of bus from input buffer */
        memcpy(&blockingMode, &payloadBufferRx[index], sizeof(boolean_T));
        index += sizeof(boolean_T);
        /* Read 8 byte blocking timeout of bus from input buffer */
        memcpy(&blockingTimeout, &payloadBufferRx[index], sizeof(double));
        index += sizeof(double);
        /*Read uORB message*/
        updated = uORB_read_step(orbData, &eventStructObj, (uint8_T*)tempPayloadBufferTx,
                                 blockingMode, blockingTimeout);
        *peripheralDataSizeResponse = orbData->o_size;
        // sending more than 255 bytes to host using writeToMATLAB function
        tempPayloadBufferTx[(*peripheralDataSizeResponse)++] = updated;
        RcvdPacket.ptrTxPayLoad = tempPayloadBufferTx;
        RcvdPacket.dataPayloadSize = *peripheralDataSizeResponse;
        writeToMATLAB((simIOpacket*)&RcvdPacket);
        payloadBufferTx = NULL;
        break;
    }
    case UORBWRITE: {
        index = 0;
        /* Read 8 byte orbData of bus from input buffer */
        memcpy(&orbData, &payloadBufferRx[index], sizeof(orbData));
        index += sizeof(orbData);
        /* Read 8 byte orbAdvertObj of bus from input buffer */
        memcpy(&orbAdvertObj, &payloadBufferRx[index], sizeof(orb_advert_t));
        index += sizeof(orb_advert_t);
        uORB_write_step(orbData, &orbAdvertObj, &payloadBufferRx[index]);
        break;
    }
    case UORBREADTERMINATE: {
        index = 0;
        memcpy(&eventStructObj, &payloadBufferRx[index], sizeof(pollfd_t));
        uORB_read_terminate(&eventStructObj);
        break;
    }
    case UORBWRITETERMINATE: {
        index = 0;
        memcpy(&orbAdvertObj, &payloadBufferRx[index], sizeof(orb_advert_t));
        uORB_write_terminate(&orbAdvertObj);
        break;
    }
        /*===========================================
         * READ PARAM
         *==========================================*/
    case PARAMINIT: {
        index = 0;
        boolean_T isString;
        double sampleTime;
        void* ParameterNameStr = 0;
        /* Read 1 byte (isString) from input buffer */
        memcpy(&isString, &payloadBufferRx[index], sizeof(boolean_T));
        index += sizeof(boolean_T);
        /* Read 8 byte (sample time) from input buffer */
        memcpy(&sampleTime, &payloadBufferRx[index], sizeof(double));
        index += sizeof(double);
        /* Rest of input buffer indicates Parameter Name */
        ParameterNameStr = &payloadBufferRx[index];
        paramHandle = MW_Init_Param(ParameterNameStr, isString, sampleTime);
        memcpy(&payloadBufferTx[0], &paramHandle, sizeof(MW_Param_Handle));
        *peripheralDataSizeResponse = sizeof(MW_Param_Handle);
        break;
    }
    case PARAMSTEP: {
        index = 0;
        MW_PARAM_DATA_TYPE ParamDataType;
        boolean_T isFloat;
        boolean_T status;
        uint8_T dataSize;
        float tempBuffer;
        /* Read the parameter handle from input buffer */
        memcpy(&paramHandle, &payloadBufferRx[index], sizeof(MW_Param_Handle));
        index += sizeof(MW_Param_Handle);
        /* Read 1 byte (isFloat) from input buffer */
        memcpy(&isFloat, &payloadBufferRx[index], sizeof(boolean_T));
        index += sizeof(boolean_T);
        if (isFloat) {
            ParamDataType = MW_SINGLE;
            dataSize = sizeof(float);
        } else {
            ParamDataType = MW_INT32;
            dataSize = sizeof(int32_t);
        }
        status = MW_Param_Step(paramHandle, ParamDataType, &tempBuffer);
        memcpy(&payloadBufferTx[0], &tempBuffer, dataSize);
        payloadBufferTx[dataSize] = status;
        *peripheralDataSizeResponse = dataSize + sizeof(boolean_T);
        break;
    }
    /*===========================================
     * Serial
     *==========================================*/
#if defined(MW_PX4_NUTTX_BUILD) /* SCI is not supported for PX4 Host Target */
    case SCIGETAVAILABLEBYTES: {
        index = 0;
        uint32_T module = 0;
        boolean_T blockingModeSCI;
        int32_T timeout;
        uint32_T availableBytes = 0;
        /* Define and set status as bus error by default */
        MW_SCI_Status_Type statusSCI = MW_SCI_BUS_ERROR;
        MW_Handle_Type sciHandle;

        /* Read 4 byte Module of bus from input buffer */
        memcpy(&module, &payloadBufferRx[index], sizeof(uint32_T));
        index += sizeof(uint32_T);
        /* Read 1 byte blockingMode of bus from input buffer */
        memcpy(&blockingModeSCI, &payloadBufferRx[index], sizeof(boolean_T));
        index += sizeof(boolean_T);
        /* Read 4 byte timeout of bus from input buffer */
        memcpy(&timeout, &payloadBufferRx[index], sizeof(int32_T));
        index += sizeof(int32_T);
        /* Retrieve the SCI bus handle from the corresponding handle map */
        sciHandle = sciMap[module];

        if ((MW_Handle_Type)NULL != sciHandle) {
            statusSCI =
                MW_SCI_GetDataBytesAvailable(sciHandle, blockingModeSCI, &availableBytes, timeout);
        }
        memcpy(&payloadBufferTx[0], &availableBytes, sizeof(uint32_T));
        payloadBufferTx[sizeof(uint32_T)] = statusSCI;
        *peripheralDataSizeResponse = sizeof(uint32_T) + sizeof(MW_SCI_Status_Type);
        break;
    }
#endif
        /*===========================================
         * PWM
         *==========================================*/
    case PWM_OPEN: {
        /* Read 1 byte isMain flag from input buffer */
        memcpy(&isMain, &payloadBufferRx[0], sizeof(boolean_T));

        /* call initialization function for PWM */
        pwm_open(&actuatorAdvertiseObj, &armAdvertiseObj);
        /* copy data to output buffer*/
        index = 0;
        memcpy(&payloadBufferTx[index], &actuatorAdvertiseObj, sizeof(actuatorAdvertiseObj));
        index += sizeof(actuatorAdvertiseObj);
        memcpy(&payloadBufferTx[index], &armAdvertiseObj, sizeof(armAdvertiseObj));
        index += sizeof(armAdvertiseObj);
        *peripheralDataSizeResponse = index;
        break;
    }
    case PWM_ARM: {
        index = 0;
        /* Read  armAdvertiseObj from input buffer */
        memcpy(&armAdvertiseObj, &payloadBufferRx[index], sizeof(armAdvertiseObj));
        index += sizeof(armAdvertiseObj);

        pwm_arm(&armAdvertiseObj);
        break;
    }
    case PWM_DISARM: {
        index = 0;
        /* Read  armAdvertiseObj from input buffer */
        memcpy(&armAdvertiseObj, &payloadBufferRx[index], sizeof(armAdvertiseObj));
        index += sizeof(armAdvertiseObj);

        pwm_disarm(&armAdvertiseObj);
        break;
    }
    case PWM_FORCEFS: {
        index = 0;
        /* Read  armAdvertiseObj from input buffer */
        memcpy(&armAdvertiseObj, &payloadBufferRx[index], sizeof(armAdvertiseObj));
        index += sizeof(armAdvertiseObj);

        pwm_forceFailsafe(&armAdvertiseObj);
        break;
    }
    case PWM_SETSERVO: {
        index = 0;
        /* Read 4 byte servoCount from input buffer */
        memcpy(&servoCount, &payloadBufferRx[index], sizeof(servoCount));
        index += sizeof(servoCount);
        /* Read 4 byte channelMask from input buffer */
        memcpy(&channelMask, &payloadBufferRx[index], sizeof(channelMask));
        index += sizeof(channelMask);
        /* Read 16 byte pwm_value from input buffer */
        memcpy(&pwm_value[0], &payloadBufferRx[index], sizeof(pwm_value));
        index += sizeof(pwm_value);
        /* Read 1 byte isMain flag from input buffer */
        memcpy(&isMain, &payloadBufferRx[index], sizeof(isMain));
        index += sizeof(isMain);
        /* Read actuatorAdvertiseObjfrom input buffer */
        memcpy(&actuatorAdvertiseObj, &payloadBufferRx[index], sizeof(actuatorAdvertiseObj));
        index += sizeof(actuatorAdvertiseObj);

        pwm_setServo(servoCount, channelMask, &pwm_value[0], isMain, &actuatorAdvertiseObj);
        break;
    }
    case PWM_RESETSERVO: {
        index = 0;
        /* Read 4 byte servoCount from input buffer */
        memcpy(&servoCount, &payloadBufferRx[index], sizeof(servoCount));
        index += sizeof(servoCount);
        /* Read 1 byte isMain flag from input buffer */
        memcpy(&isMain, &payloadBufferRx[index], sizeof(isMain));
        index += sizeof(isMain);
        /* Read actuatorAdvertiseObjfrom input buffer */
        memcpy(&actuatorAdvertiseObj, &payloadBufferRx[index], sizeof(actuatorAdvertiseObj));
        index += sizeof(actuatorAdvertiseObj);

        pwm_resetServo(servoCount, isMain, &actuatorAdvertiseObj);
        break;
    }
    case PWM_CLOSE: {
        index = 0;
        /* Read 4 byte servoCount from input buffer */
        memcpy(&servoCount, &payloadBufferRx[index], sizeof(servoCount));
        index += sizeof(servoCount);
        /* Read actuatorAdvertiseObjfrom input buffer */
        memcpy(&actuatorAdvertiseObj, &payloadBufferRx[index], sizeof(actuatorAdvertiseObj));
        index += sizeof(actuatorAdvertiseObj);
        /* Read armAdvertiseObj input buffer */
        memcpy(&armAdvertiseObj, &payloadBufferRx[index], sizeof(armAdvertiseObj));
        index += sizeof(armAdvertiseObj);

        pwm_close(servoCount, &actuatorAdvertiseObj, &armAdvertiseObj);
        break;
    }
    /*===========================================
     * CAN
     *==========================================*/
#if defined(MW_PX4_NUTTX_BUILD) && \
    !defined(MW_PX4_CAN_DISABLE) /* CAN is not supported for PX4 Host Target */
    case CAN_OPEN: {
        index = 0;
        uint32_T baudRate;
        uint8_T mode;
        /* Read 1 byte canModule from input buffer */
        memcpy(&CANModule, &payloadBufferRx[index], sizeof(CANModule));
        index += sizeof(CANModule);
        /* Read 4 byte baudRate from input buffer */
        memcpy(&baudRate, &payloadBufferRx[index], sizeof(baudRate));
        index += sizeof(baudRate);
        /* Read 1 byte mode from input buffer */
        memcpy(&mode, &payloadBufferRx[index], sizeof(mode));
        index += sizeof(mode);
        errorStatus = MW_CAN_Open(CANModule, baudRate, mode);
        /* copy errorStatus to output buffer*/
        memcpy(&payloadBufferTx[0], &errorStatus, sizeof(errorStatus));
        *peripheralDataSizeResponse = sizeof(errorStatus);
        break;
    }
    case CAN_CLOSE: {
        index = 0;
        /* Read 1 byte canModule from input buffer */
        memcpy(&CANModule, &payloadBufferRx[index], sizeof(CANModule));
        index += sizeof(CANModule);
        MW_CAN_Close(CANModule);
        break;
    }
    case CAN_ASSIGNBUFFERID: {
        index = 0;
        /* Read 1 byte canModule from input buffer */
        memcpy(&CANModule, &payloadBufferRx[index], sizeof(CANModule));
        index += sizeof(CANModule);
        /* Read 4 byte id from input buffer */
        memcpy(&id, &payloadBufferRx[index], sizeof(id));
        index += sizeof(id);
        /* Read 1 byte idType from input buffer */
        memcpy(&idType, &payloadBufferRx[index], sizeof(idType));
        index += sizeof(idType);
        MW_CAN_AssignGlobalBufferForID(CANModule, id, idType);
        break;
    }
    case CAN_TRANSMIT: {
        index = 0;
        /* Read 1 byte canModule from input buffer */
        memcpy(&CANModule, &payloadBufferRx[index], sizeof(CANModule));
        index += sizeof(CANModule);
        /* Read 4 byte id from input buffer */
        memcpy(&id, &payloadBufferRx[index], sizeof(id));
        index += sizeof(id);
        /* Read 1 byte idType from input buffer */
        memcpy(&idType, &payloadBufferRx[index], sizeof(idType));
        index += sizeof(idType);
        /* Read 1 byte remote from input buffer */
        memcpy(&remote, &payloadBufferRx[index], sizeof(remote));
        index += sizeof(remote);
        /* Read 1 byte length from input buffer */
        memcpy(&length, &payloadBufferRx[index], sizeof(length));
        index += sizeof(length);
        errorStatus =
            MW_CAN_TransmitMessage(CANModule, &payloadBufferRx[index], id, idType, remote, length);
        /* copy errorStatus to output buffer*/
        memcpy(&payloadBufferTx[0], &errorStatus, sizeof(errorStatus));
        *peripheralDataSizeResponse = sizeof(errorStatus);
        break;
    }
    case CAN_RECEIVEBYID: {
        index = 0;
        /* Read 1 byte canModule from input buffer */
        memcpy(&CANModule, &payloadBufferRx[index], sizeof(CANModule));
        index += sizeof(CANModule);
        /* Read 4 byte id from input buffer */
        memcpy(&id, &payloadBufferRx[index], sizeof(id));
        index += sizeof(id);
        /* Read 1 byte idType from input buffer */
        memcpy(&idType, &payloadBufferRx[index], sizeof(idType));
        index += sizeof(idType);
        errorStatus = MW_CAN_ReceiveMessages_By_ID(CANModule, &payloadBufferTx[0], id, idType,
                                                   &remote, &length);
        index = 8; // Length of CAN data
        /* copy remote to output buffer*/
        memcpy(&payloadBufferTx[index], &remote, sizeof(remote));
        index += sizeof(remote);
        /* copy length to output buffer*/
        memcpy(&payloadBufferTx[index], &length, sizeof(length));
        index += sizeof(length);
        /* copy errorStatus to output buffer*/
        memcpy(&payloadBufferTx[index], &errorStatus, sizeof(errorStatus));
        *peripheralDataSizeResponse = index + sizeof(errorStatus);
        break;
    }
    case CAN_RECEIVEMESSAGE: {
        index = 0;
        /* Read 1 byte canModule from input buffer */
        memcpy(&CANModule, &payloadBufferRx[index], sizeof(CANModule));
        index += sizeof(CANModule);
        errorStatus =
            MW_CAN_ReceiveMessage(CANModule, &payloadBufferTx[0], &id, &idType, &remote, &length);
        index = 8; // Length of CAN data
        /* copy id to output buffer*/
        memcpy(&payloadBufferTx[index], &id, sizeof(id));
        index += sizeof(id);
        /* copy idType to output buffer*/
        memcpy(&payloadBufferTx[index], &idType, sizeof(idType));
        index += sizeof(idType);
        /* copy remote to output buffer*/
        memcpy(&payloadBufferTx[index], &remote, sizeof(remote));
        index += sizeof(remote);
        /* copy length to output buffer*/
        memcpy(&payloadBufferTx[index], &length, sizeof(length));
        index += sizeof(length);
        /* copy errorStatus to output buffer*/
        memcpy(&payloadBufferTx[index], &errorStatus, sizeof(errorStatus));
        *peripheralDataSizeResponse = index + sizeof(errorStatus);
        break;
    }
#endif
        /*===========================================
         * PX4 TIME STAMP
         *==========================================*/
    case GET_HRTABSTIME: {
        index = 0;
        uint64_T hrtTimeStamp;
        hrtTimeStamp = hrt_absolute_time();
        memcpy(&payloadBufferTx[index], &hrtTimeStamp, sizeof(hrtTimeStamp));
        *peripheralDataSizeResponse = sizeof(hrtTimeStamp);
        break;
    }
    default:
        break;
    }
}
