#include "WRPosition.h"
#include <iostream>

struct wr_state WRState = { 0 };

SbgErrorCode onLogReceived(SbgEComHandle *pHandle,
			  SbgEComClass msgClass,
			  SbgEComMsgId msg,
			  const SbgBinaryLogData *pLogData,
			  void *pUserArg) {
  switch (msg) {
  case SBG_ECOM_LOG_STATUS:
    //std::cerr << "Received status" << std::endl;
    break;
  case SBG_ECOM_LOG_UTC_TIME:
    //std::cerr << "Received utc" << std::endl;
    WRState.time = pLogData->utcData.timeStamp;
    break;
  case SBG_ECOM_LOG_GPS1_VEL:
    //std::cerr << "Received velocity" << std::endl;
    WRState.velocity = sqrt(pow(pLogData->gpsVelData.velocity[0], 2) +
			     pow(pLogData->gpsVelData.velocity[1], 2));
    break;
  case SBG_ECOM_LOG_GPS1_POS:
    //std::cerr << "Received position" << std::endl;
    WRState.position = Eigen::Vector2d(pLogData->gpsPosData.latitude,
				       pLogData->gpsPosData.longitude);
    break;
  case SBG_ECOM_LOG_GPS1_HDT:
    //std::cerr << "Received heading" << std::endl;
    WRState.heading = pLogData->gpsHdtData.heading;
    break;
  }
}

void getWRPosition(std::string dev) {
  SbgEComHandle comHandle;
  SbgErrorCode sbgErr;
  SbgInterface sbgInterface;
  int32 retValue = 0;
  SbgEComDeviceInfo devInfo;

  sbgErr = sbgInterfaceSerialCreate(&sbgInterface, dev.c_str(), 921600);
  
  if (sbgErr == SBG_NO_ERROR) {
    sbgErr = sbgEComInit(&comHandle, &sbgInterface);

    // Configure for usage with UART
    const SbgEComInterfaceConf sbgConf = {
      115200, // Baud rate
      SBG_ECOM_UART_MODE_232
    };

    sbgEComCmdInterfaceSetUartConf(&comHandle, SBG_ECOM_IF_COM_A, &sbgConf);
    if (sbgErr != SBG_NO_ERROR) {
	std::cerr << "Unable to do serial config, err="
		  << std::to_string(sbgErr) << std::endl;
	exit(1);
    }
    
    if (sbgErr == SBG_NO_ERROR) {
      // Not able to send to GPS right now, only to read.
      //sbgErr = sbgEComCmdGetInfo(&comHandle, &devInfo);
      
      //if (sbgErr == SBG_NO_ERROR) {
	if (true) {
	  //std::cerr << "Device " << devInfo.serialNumber << " found!" << std::endl;

	// Do some configuration

	sbgEComSetReceiveLogCallback(&comHandle, onLogReceived, NULL);

	while (1) {
	  sbgErr = sbgEComHandle(&comHandle);

	  if (sbgErr == SBG_NOT_READY) {
	    sbgSleep(1);
	  }
	  else {
	    std::cerr << "Error! Oh NO!" << std::endl;
	  }
	}
	sbgEComClose(&comHandle);
      }
      else {
	std::cerr << "Unable to get device information, err="
		  << std::to_string(sbgErr) << std::endl;
	exit(1);
	}
    }
    else {
      std::cerr << "Unable to initialize the sbgECom library." << std::endl;
      exit(1);
    }

    sbgInterfaceSerialDestroy(&sbgInterface);
  }
  else {
    std::cerr << "Unable to create interface to Ellipse." << std::endl;
    exit(1);
  }
}
