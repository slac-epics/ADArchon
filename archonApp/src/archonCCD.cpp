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
      checkStatus(mDrv->clear_acquisition(), "Error: Unable to stop aquisition");
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
    sprintf(error, "ERROR: %s.", message);
    throw std::string(error);
  } else {
    return status;
  }
}

