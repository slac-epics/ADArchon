/**
 * Area Detector driver for the Archon CCD.
 *
 * @author Daniel Damiani
 * @date Aug 2022
 */

#include <stdio.h>
#include <string.h>
#include <string>
#include <errno.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsString.h>
#include <iocsh.h>
#include <epicsExit.h>
#include <ADDriver.h>

#include "archon.h"

#include <epicsExport.h>
#include "archonCCD.h"

#define DRIVER_VERSION      0
#define DRIVER_REVISION     1
#define DRIVER_MODIFICATION 0

static const char *driverName = "archonCCD";

static void exitHandler(void *drvPvt)
{
  ArchonCCD *pArchonCCD = (ArchonCCD *) drvPvt;
  delete pArchonCCD;
}

ArchonCCD::ArchonCCD(const char *portName, const char *filePath, const char *cameraAddr, int cameraPort,
                     int maxBuffers, size_t maxMemory, int priority, int stackSize) :
  ADDriver(portName, 1, 0, maxBuffers, maxMemory,
           asynEnumMask, asynEnumMask,
           ASYN_CANBLOCK, 1, priority, stackSize),
  mDrv(new Pds::Archon::Driver(cameraAddr, cameraPort)),
  mExiting(false),
  mExited(0),
  mInitOK(false)
{
  int status = asynSuccess;

  static const char *functionName = "ArchonCCD";

  // Add an EPICS exit handler
  epicsAtExit(exitHandler, this);

  // Create the epicsEvent for signaling to the status task when parameters should have changed.
  // This will cause it to do a poll immediately, rather than wait for the poll time period.
  this->statusEvent = epicsEventMustCreate(epicsEventEmpty);
  if (!this->statusEvent) {
    printf("%s:%s epicsEventCreate failure for start event\n", driverName, functionName);
    return;
  }

  // Use this to signal the data acquisition task that acquisition has started.
  this->dataEvent = epicsEventMustCreate(epicsEventEmpty);
  if (!this->dataEvent) {
    printf("%s:%s epicsEventCreate failure for data event\n", driverName, functionName);
    return;
  }

  // Initialize camera
  try {
    checkStatus(mDrv->configure(filePath), "Unable to apply configuration file");

    // Get the current temperature
    checkStatus(mDrv->fetch_status(), "Unable to fetch status info");
    const Pds::Archon::Status& status = mDrv->status();
    double temperature = status.backplane_temp();
    printf("%s:%s: current backplane temperature is %f C\n", driverName, functionName, temperature);
    setDoubleParam(ADTemperature, temperature);
    callParamCallbacks();
  } catch (const std::string &e) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: %s\n",
              driverName, functionName, e.c_str());
    return;
  }

  printf("Archon CCD initialized OK!\n");
  mInitOK = true;
}

ArchonCCD::~ArchonCCD()
{
  static const char *functionName = "~ArchonCCD";
  asynStatus status;

  mExiting = true;
  this->lock();
  printf("%s::%s Shutdown and disconnect...\n", driverName, functionName);
  try {
    if (mDrv->acquisition_mode() != Pds::Archon::Stopped) {
      checkStatus(mDrv->clear_acquisition(), "Unable to stop aquisition");
    }
    epicsEventSignal(dataEvent);
    delete mDrv;
  } catch (const std::string &e) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: %s\n",
              driverName, functionName, e.c_str());
    status = asynError;
  }
  this->unlock();
  while ((mExited < 2) && (status != asynError))
    epicsThreadSleep(0.2);
}

bool ArchonCCD::checkStatus(bool status, const char *message)
{
  char error[256];
  if (!status) {
    sprintf(error, "Error - %s!", message);
    throw std::string(error);
  } else {
    return status;
  }
}

/** IOC shell configuration command for Archon driver
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] filePath The path to the Archon dectector configuration file.
  * \param[in] cameraSerial The hostname/ip address number of the desired camera.
  * \param[in] cameraPort The tcp port number which the camera is listening for connections on.
  * \param[in] maxBuffers The maximum number of NDArray buffers that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in] maxMemory The maximum amount of memory that the NDArrayPool for this driver is
  *            allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  *            files on Windows, but must be a valid path on Linux.
  * \param[in] priority The thread priority for the asyn port driver thread
  * \param[in] stackSize The stack size for the asyn port driver thread
  */
extern "C" {
int archonCCDConfig(const char *portName, const char *filePath, const char *cameraAddr, int cameraPort,
                    int maxBuffers, size_t maxMemory, int priority, int stackSize)
{
  /*Instantiate class.*/
  new ArchonCCD(portName, filePath, cameraAddr, cameraPort, maxBuffers, maxMemory, priority, stackSize);
  return(asynSuccess);
}


/* Code for iocsh registration */

/* archonCCDConfig */
static const iocshArg archonCCDConfigArg0 = {"Port name", iocshArgString};
static const iocshArg archonCCDConfigArg1 = {"filePath", iocshArgString};
static const iocshArg archonCCDConfigArg2 = {"cameraAddr", iocshArgString};
static const iocshArg archonCCDConfigArg3 = {"cameraPort", iocshArgInt};
static const iocshArg archonCCDConfigArg4 = {"maxBuffers", iocshArgInt};
static const iocshArg archonCCDConfigArg5 = {"maxMemory", iocshArgInt};
static const iocshArg archonCCDConfigArg6 = {"priority", iocshArgInt};
static const iocshArg archonCCDConfigArg7 = {"stackSize", iocshArgInt};
static const iocshArg * const archonCCDConfigArgs[] =  {&archonCCDConfigArg0,
                                                       &archonCCDConfigArg1,
                                                       &archonCCDConfigArg2,
                                                       &archonCCDConfigArg3,
                                                       &archonCCDConfigArg4,
                                                       &archonCCDConfigArg5,
                                                       &archonCCDConfigArg6,
                                                       &archonCCDConfigArg7};

static const iocshFuncDef configArchonCCD = {"archonCCDConfig", 8, archonCCDConfigArgs};
static void configArchonCCDCallFunc(const iocshArgBuf *args)
{
    archonCCDConfig(args[0].sval, args[1].sval, args[2].sval, args[3].ival,
                   args[4].ival, args[5].ival, args[6].ival, args[7].ival);
}

static void archonCCDRegister(void)
{

    iocshRegister(&configArchonCCD, configArchonCCDCallFunc);
}

epicsExportRegistrar(archonCCDRegister);
}
