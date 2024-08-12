/* Copyright 2020-2023 The MathWorks, Inc. */


#include "nuttxinitialize.h"
#include "MW_PX4_TaskControl.h"

#ifdef __cplusplus
extern "C" { /* sbcheck:ok:extern_c needed because of C compatibility*/
#endif
#include "IO_include.h"
#include "IO_server.h"
#include "IO_packet.h"


static uint8_T PayloadBufferRxBackground[PAYLOAD_SIZE];
static uint8_T PayloadBufferTxBackground[PAYLOAD_SIZE];
// Hook for configuring the base rate at which the scheduler should be called and starts the
// scheduler
void configureScheduler(float SchedulerBaseRate) {}
// Hook to enable the scheduler interrupt
void enableSchedulerInterrupt(void) {}
// Hook to disable the scheduler interrupt
void disableSchedulerInterrupt(void) {}
// Hook to enable the global interrupt on the target
void enableGlobalInterrupt(void) {}
// Hook to disable the global interrupt on the target
void disableGlobalInterrupt(void) {}
// Hook to stop the scheduler
void stopScheduler(void) {}
#ifdef __cplusplus
}
#endif
void* baseRateTask(void* arg);

px4_sem_t baserateTaskSem;
px4_sem_t stopSem;
pthread_t baseRateThread;
int terminatingmodel = 0;
extern struct hrt_call BaseRate_HRT;

void* baseRateTask(void* arg) {
    while (1) {
        // sem_wait(&baserateTaskSem);
        //  g2292174 fix - simulation errors out while using 'actuator_outputs' uORB message
        usleep(1000);
        server((uint8_T*)&PayloadBufferRxBackground, (uint8_T*)&PayloadBufferTxBackground, 1);
    }
}
int px4_simulink_app_task_main(int argc, char* argv[]) {

#ifndef HOST_TARGET
    // disable MAVLink over \dev\ttyACM0
    px4_simulink_app_control_MAVLink();
#endif

    // IO server init
    ioServerInit();

    // Call RTOS Initialization function
    nuttxRTOSInit(0.005, 0);

    px4_sem_wait(&stopSem);
    hrt_cancel(&BaseRate_HRT);
    return 0;
}
