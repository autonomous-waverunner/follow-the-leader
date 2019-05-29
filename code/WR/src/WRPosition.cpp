#include "WRPosition.h"
#include <iostream>

struct wr_state WRState = { 0 };

std::string stat(uint32 f, uint32 m) {
  return std::to_string((f & m) - ((f & m) ? (m - 1) : 0));
}

SbgErrorCode onLogReceived(SbgEComHandle *pHandle,
			  SbgEComClass msgClass,
			  SbgEComMsgId msg,
			  const SbgBinaryLogData *pLogData,
			  void *pUserArg) {
  switch (msg) {
  case SBG_ECOM_LOG_STATUS:
    gps_log << "STA " << std::endl;
    /*    std::cerr << "Main power: " << stat(pLogData->statusData.generalStatus, SBG_ECOM_GENERAL_MAIN_POWER_OK)
	      << " GPS power: " << stat(pLogData->statusData.generalStatus, SBG_ECOM_GENERAL_GPS_POWER_OK)
	      << " Settings: " << stat(pLogData->statusData.generalStatus, SBG_ECOM_GENERAL_SETTINGS_OK)
	      << " Temp: " << stat(pLogData->statusData.generalStatus, SBG_ECOM_GENERAL_TEMPERATURE_OK)
	      << " Log: " << stat(pLogData->statusData.generalStatus, SBG_ECOM_GENERAL_DATALOGGER_OK)
	      << " CPU: " << stat(pLogData->statusData.generalStatus, SBG_ECOM_GENERAL_CPU_OK)
	      << std::endl;
    std::cerr << "Port A OK: " << stat(pLogData->statusData.comStatus, SBG_ECOM_PORTA_VALID)
	      << " Port A TX OK: " << stat(pLogData->statusData.comStatus, SBG_ECOM_PORTA_TX_OK)
	      << " Port A RX OK: " << stat(pLogData->statusData.comStatus, SBG_ECOM_PORTA_RX_OK)
	      << std::endl;
    std::cerr << "GPS1 pos: " << stat(pLogData->statusData.aidingStatus, SBG_ECOM_AIDING_GPS1_POS_RECV)
	      << " vel: " << stat(pLogData->statusData.aidingStatus, SBG_ECOM_AIDING_GPS1_VEL_RECV)
	      << " hdt: " << stat(pLogData->statusData.aidingStatus, SBG_ECOM_AIDING_GPS1_HDT_RECV)
	      << " utc: " << stat(pLogData->statusData.aidingStatus, SBG_ECOM_AIDING_GPS1_UTC_RECV)
	      << " GPS2 pos: " << stat(pLogData->statusData.aidingStatus, SBG_ECOM_AIDING_GPS2_POS_RECV)
	      << " vel: " << stat(pLogData->statusData.aidingStatus, SBG_ECOM_AIDING_GPS2_VEL_RECV)
	      << " hdt: " << stat(pLogData->statusData.aidingStatus, SBG_ECOM_AIDING_GPS2_HDT_RECV)
	      << " utc: " << stat(pLogData->statusData.aidingStatus, SBG_ECOM_AIDING_GPS2_UTC_RECV)
	      << std::endl;*/
    break;
  case SBG_ECOM_LOG_UTC_TIME:
    gps_log << "UTC "
	    << pLogData->utcData.timeStamp
	    << std::endl;
    WRState.time = pLogData->utcData.timeStamp;
    break;
  case SBG_ECOM_LOG_GPS1_VEL:
    gps_log << "VEL "
	    << sqrt(pow(pLogData->gpsVelData.velocity[0], 2) +
		    pow(pLogData->gpsVelData.velocity[1], 2))
	    << std::endl;
    WRState.velocity = sqrt(pow(pLogData->gpsVelData.velocity[0], 2) +
			     pow(pLogData->gpsVelData.velocity[1], 2));
    break;
  case SBG_ECOM_LOG_GPS1_POS:
    gps_log << "POS "
	    << pLogData->gpsPosData.latitude << "," << pLogData->gpsPosData.longitude
	    << std::endl;
    //    std::cerr << "GPS POS Sats: " << std::to_string(pLogData->gpsPosData.numSvUsed)
    projPJ pj_utm, pj_latlong;
    double x, y;
  
    if (!(pj_utm = pj_init_plus("+proj=utm +zone=32")) )
      throw 255;
    if (!(pj_latlong = pj_init_plus("+proj=latlong +ellps=WGS84")) )
      throw 255;

    x = pLogData->gpsPosData.latitude * DEG_TO_RAD;
    y = pLogData->gpsPosData.longitude * DEG_TO_RAD;
    if (pj_transform(pj_latlong, pj_utm, 1, 1, &x, &y, NULL ))
      throw 255;

    WRState.position = Eigen::Vector2d(x, y);
    break;
  case SBG_ECOM_LOG_GPS1_HDT:
    gps_log << "HDT "
	    << pLogData->gpsHdtData.heading
	    << std::endl;
    
    double heading;
    if (pLogData->gpsHdtData.heading > 180)
      heading = pLogData->gpsHdtData.heading - 360;
    else
      heading = pLogData->gpsHdtData.heading;
    WRState.heading = heading * 3.1415926535 / 180;
    break;
  case SBG_ECOM_LOG_EKF_NAV:
    gps_log << "EKF POS "
	    << pLogData->ekfNavData.position[0] << ", "
	    << pLogData->ekfNavData.position[1] << ", "
	    << pLogData->ekfNavData.position[2]
	    << std::endl;
    gps_log << "EKF VEL "
	    << pLogData->ekfNavData.velocity[0] << ", "
	    << pLogData->ekfNavData.velocity[1] << ", "
	    << pLogData->ekfNavData.velocity[2]
	    << std::endl;
    break;
  }
}

void getWRPosition(std::string dev) {
  SbgEComHandle comHandle;
  SbgErrorCode sbgErr;
  SbgInterface sbgInterface;
  //int32 retValue = 0;
  //SbgEComDeviceInfo devInfo;

  /* The baud rate is limited to 115200 due to the RS232 <-> TTL 5V conversion;
     higher baud rates result in corrupt data. One possible solution for this is
     to change the MAX202 chip to a MAX3232 which have correct logic levels */
  sbgErr = sbgInterfaceSerialCreate(&sbgInterface, dev.c_str(), 115200);
  
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
