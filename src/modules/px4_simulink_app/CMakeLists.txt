## This cmakelist.txt file was generated from
## the UAV Toolbox Support Package for PX4 Autopilots
#
# Copyright 2020-2023 The MathWorks, Inc.

set(MAVLINK_LIBRARY_DIR "${CMAKE_BINARY_DIR}/mavlink")
set(MAVLINK_DIALECT_UAVIONIX "uAvionix")

add_definitions(
-DMODEL=connectedIO -DNUMST=1 -DHAVESTDIO -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0 -DCLASSIC_INTERFACE=0 -DALLOCATIONFCN=0 -DTID01EQ=0 -DTERMFCN=1 -DONESTEPFCN=1 -DMAT_FILE=0 -DMULTI_INSTANCE_CODE=0 -DINTEGER_CODE=0 -DMT=0 -DPX4 -DSTACK_SIZE=64 -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__ -DRT -DNULL=0 -DPX4_CONNECTEDIO -DHOST_TARGET -DMW_PX4_POSIX_BUILD)

px4_add_module(
    MODULE modules__px4_simulink_app
    MAIN px4_simulink_app
    STACK_MAIN 2000
SRCS
   MW_uORB_Read.cpp
   MW_uORB_Write.cpp
   MW_PX4_TaskControl.cpp
   nuttxinitialize.cpp
   MW_ParameterRead.cpp
   MW_PX4_PWM.cpp
   uORBMetaData.c
   customFunction.cpp
   IO_main.cpp
   IO_checksum.c
   IO_debug.c
   IO_packet.c
   IO_server.c
   IO_standardperipherals.c
   IO_utilities.c
   PeripheralToHandle.c
   MW_PX4_rtiostream_tcpip.cpp
   MW_target_hardware_resources.h
   MW_uORB_busstruct_conversion.h
   connectedIO.h
   connectedIO_private.h
   connectedIO_types.h
   peripheralIncludes.h
   uORBMetaData.h
   MW_PIL_custom_RTOS_header.h
   MW_PX4_I2C.h
   MW_PX4_PWM.h
   MW_PX4_SCI.h
   MW_PX4_TaskControl.h
   MW_Parameter.h
   MW_ParameterRead.h
   MW_uORB_Init.h
   MW_uORB_Read.h
   MW_uORB_Write.h
   nuttxinitialize.h
   profiler_timer.h
   MW_custom_RTOS_header.h
   customFunction.h
   rtwtypes.h
   MW_AnalogIn.h
   MW_CAN.h
   MW_ETH.h
   MW_I2C.h
   MW_PWM.h
   MW_SCI.h
   MW_SPI.h
   MW_SVD.h
   MW_digitalIO.h
   IO_addOn.h
   IO_checksum.h
   IO_checksumPrereq.h
   IO_debug.h
   IO_include.h
   IO_packet.h
   IO_peripheralInclude.h
   IO_requestID.h
   IO_server.h
   IO_standardperipherals.h
   IO_utilities.h
   IO_wrapperAnalogInput.h
   IO_wrapperDigitalIO.h
   IO_wrapperI2C.h
   IO_wrapperPWM.h
   IO_wrapperSCI.h
   IO_wrapperSPI.h
   PeripheralHandleMapType.h
   PeripheralToHandle.h
   hardware.h
   rtiostream.h
    COMPILE_FLAGS
       -fpermissive
       -Wno-narrowing
	   -Wno-address-of-packed-member # Needed by mavlink module
	   -Wno-cast-align # Needed by mavlink module  
    INCLUDES
       ${PX4_SOURCE_DIR}/src/modules/px4_simulink_app
       ${PX4_SOURCE_DIR}/src/modules/mavlink
 	   ${MAVLINK_LIBRARY_DIR}
	   ${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}	 
	   ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}	   
)

get_target_property(PX4_SL_APP_COMPILE_FLAGS modules__px4_simulink_app COMPILE_OPTIONS)
list(REMOVE_ITEM PX4_SL_APP_COMPILE_FLAGS -Werror)
set_target_properties(modules__px4_simulink_app PROPERTIES COMPILE_OPTIONS "${PX4_SL_APP_COMPILE_FLAGS}")

# LocalWords:  UAVIONIX Avionix
