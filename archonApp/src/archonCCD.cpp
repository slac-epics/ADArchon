/**
 * Area Detector driver for the Archon CCD.
 *
 * @author Daniel Damiani
 * @date Aug 2022
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <errno.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsString.h>
#include <epicsMath.h>
#include <iocsh.h>
#include <epicsExit.h>
#include <ADDriver.h>

#include "archon.h"
#include "archonVersion.h"

#include <epicsExport.h>
#include "archonCCD.h"

#define DRIVER_VERSION      0
#define DRIVER_REVISION     1
#define DRIVER_MODIFICATION 0

/* Max size for enum strings */
#define MAX_ENUM_STRING_SIZE 25

#define sizeofArray(arr) sizeof(arr) / sizeof(arr[0])

static const char *driverName = "archonCCD";

const epicsInt32 ArchonCCD::ABNone = 0;
const epicsInt32 ArchonCCD::ABX12 = 1;
const epicsInt32 ArchonCCD::ABX16 = 2;
const epicsInt32 ArchonCCD::ABUnknown = 3;

const epicsInt32 ArchonCCD::APUnknown = 0;
const epicsInt32 ArchonCCD::APNotConfigured = 1;
const epicsInt32 ArchonCCD::APOff = 2;
const epicsInt32 ArchonCCD::APIntermediate = 3;
const epicsInt32 ArchonCCD::APOn = 4;
const epicsInt32 ArchonCCD::APStandby = 5;

const epicsInt32 ArchonCCD::ARImage = 0;
const epicsInt32 ArchonCCD::ARFullVerticalBinning = 1;

const epicsInt32 ArchonCCD::ABNV4 = -4;
const epicsInt32 ArchonCCD::ABNV3 = -3;
const epicsInt32 ArchonCCD::ABNV2 = -2;
const epicsInt32 ArchonCCD::ABNV1 = -1;
const epicsInt32 ArchonCCD::ABPV1 = 1;
const epicsInt32 ArchonCCD::ABPV2 = 2;
const epicsInt32 ArchonCCD::ABPV3 = 3;
const epicsInt32 ArchonCCD::ABPV4 = 4;

//Definitions of static class data members
const ArchonCCD::ArchonEnumInfo ArchonCCD::BackplaneTypeEnums[] = {
  {"None",    ABNone,     epicsSevNone},
  {"X12",     ABX12,      epicsSevNone},
  {"X16",     ABX16,      epicsSevNone},
  {"Unknown", ABUnknown,  epicsSevNone},
};

const ArchonCCD::ArchonEnumInfo ArchonCCD::PowerModeEnums[] = {
  {"Unknown",       APUnknown,        epicsSevInvalid},
  {"NotConfigured", APNotConfigured,  epicsSevMinor},
  {"Off",           APOff,            epicsSevNone},
  {"Intermediate",  APIntermediate,   epicsSevNone},
  {"On",            APOn,             epicsSevNone},
  {"Standby",       APStandby,        epicsSevNone},
};

const ArchonCCD::ArchonEnumInfo ArchonCCD::ReadOutModeEnums[] = {
  {"Image", ARImage,                epicsSevNone},
  {"FVB",   ARFullVerticalBinning,  epicsSevNone},
};

const ArchonCCD::ArchonEnumInfo ArchonCCD::BiasChannelEnums[] = {
  {"NV4", ABNV4, epicsSevNone},
  {"NV3", ABNV3, epicsSevNone},
  {"NV2", ABNV2, epicsSevNone},
  {"NV1", ABNV1, epicsSevNone},
  {"PV1", ABPV1, epicsSevNone},
  {"PV2", ABPV2, epicsSevNone},
  {"PV3", ABPV3, epicsSevNone},
  {"PV4", ABPV4, epicsSevNone},
};

const ArchonCCD::ArchonEnumSet ArchonCCD::ArchonEnums[] = {
  {BackplaneTypeEnums,
   sizeofArray(BackplaneTypeEnums),
   ArchonBackplaneTypeString},
  {PowerModeEnums,
   sizeofArray(PowerModeEnums),
   ArchonPowerModeString},
  {ReadOutModeEnums,
   sizeofArray(ReadOutModeEnums),
   ArchonReadOutModeString},
  {BiasChannelEnums,
   sizeofArray(BiasChannelEnums),
   ArchonBiasChanString}
};

const size_t ArchonCCD::ArchonEnumsSize = sizeofArray(ArchonCCD::ArchonEnums);

static unsigned archonTimeConvert(double time_in_sec)
{
  unsigned time_in_ms = 0;
  if (time_in_sec > 0.0) {
    time_in_ms = (unsigned) (time_in_sec*1000);
  }

  return time_in_ms;
}

static void archonStatusTaskC(void *drvPvt)
{
  ArchonCCD *pPvt = (ArchonCCD *)drvPvt;

  pPvt->statusTask();
}


static void archonDataTaskC(void *drvPvt)
{
  ArchonCCD *pPvt = (ArchonCCD *)drvPvt;

  pPvt->dataTask();
}

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
  mDrvMutex(new epicsMutex()),
  mStopping(false),
  mExiting(false),
  mExited(0),
  mFrameBuffer(NULL),
  mFrameBufferSize(0),
  mInitOK(false)
{
  int status = asynSuccess;
  int binX=1, binY=1, minX=0, minY=0, sizeX, sizeY;
  unsigned batch=0;
  unsigned preframeClear=0;
  unsigned idleClear=1;
  unsigned preframeSkip=22;
  unsigned clkat = 2000;
  unsigned clkst = 30;
  unsigned clkstm1 = 29;
  float biasSetpoint = -40.0;
  float biasVoltage, biasCurrent;
  double framePollingPeriod = 0.001;
  int biasChan = ABNV1;
  char tempString[256];
  std::string serialNumber;
  std::string firmwareVersion;
  unsigned backplaneType;
  unsigned backplaneRev;
  unsigned sampleMode;
  unsigned lineScanMode;
  unsigned frameSize;
  unsigned bitsPerPixel;
  unsigned totalTaplines;
  unsigned activeTaplines;
  Pds::Archon::PowerMode powerMode = Pds::Archon::Unknown;

  static const char *functionName = "ArchonCCD";

  // Add an EPICS exit handler
  epicsAtExit(exitHandler, this);

  createParam(ArchonMessageString,          asynParamOctet,   &ArchonMessage);
  createParam(ArchonBackplaneTypeString,    asynParamInt32,   &ArchonBackplaneType);
  createParam(ArchonBackplaneRevString,     asynParamInt32,   &ArchonBackplaneRev);
  createParam(ArchonPowerModeString,        asynParamInt32,   &ArchonPowerMode);
  createParam(ArchonPowerSwitchString,      asynParamInt32,   &ArchonPowerSwitch);
  createParam(ArchonReadOutModeString,      asynParamInt32,   &ArchonReadOutMode);
  createParam(ArchonNumBatchFramesString,   asynParamInt32,   &ArchonNumBatchFrames);
  createParam(ArchonLineScanModeString,     asynParamInt32,   &ArchonLineScanMode);
  createParam(ArchonPreFrameClearString,    asynParamInt32,   &ArchonPreFrameClear);
  createParam(ArchonIdleClearString,        asynParamInt32,   &ArchonIdleClear);
  createParam(ArchonPreFrameSkipString,     asynParamInt32,   &ArchonPreFrameSkip);
  createParam(ArchonNonIntTimeString,       asynParamFloat64, &ArchonNonIntTime);
  createParam(ArchonClockAtString,          asynParamInt32,   &ArchonClockAt);
  createParam(ArchonClockStString,          asynParamInt32,   &ArchonClockSt);
  createParam(ArchonClockStm1String,        asynParamInt32,   &ArchonClockStm1);
  createParam(ArchonBiasChanString,         asynParamInt32,   &ArchonBiasChan);
  createParam(ArchonBiasSetpointString,     asynParamFloat64, &ArchonBiasSetpoint);
  createParam(ArchonBiasSwitchString,       asynParamInt32,   &ArchonBiasSwitch);
  createParam(ArchonBiasVoltageString,      asynParamFloat64, &ArchonBiasVoltage);
  createParam(ArchonBiasCurrentString,      asynParamFloat64, &ArchonBiasCurrent);
  createParam(ArchonFramePollPeriodString,  asynParamFloat64, &ArchonFramePollPeriod);
  createParam(ArchonTotalTaplinesString,    asynParamInt32,   &ArchonTotalTaplines);
  createParam(ArchonActiveTaplinesString,   asynParamInt32,   &ArchonActiveTaplines);
  createParam(ArchonPixelsPerTapString,     asynParamInt32,   &ArchonPixelsPerTap);
  createParam(ArchonConfigFileString,       asynParamOctet,   &ArchonConfigFile);

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
    printf("%s:%s: configured camera using config file %s\n", driverName, functionName, filePath);

    setStringParam(ArchonMessage, "Camera successfully initialized.");
    setStringParam(ArchonConfigFile, filePath);

    // Get system info
    checkStatus(mDrv->fetch_system(), "Unable to fetch system info");
    const Pds::Archon::System& system = mDrv->system();
    backplaneType = system.type();
    backplaneRev = system.rev();
    serialNumber = system.id();
    firmwareVersion = system.version();

    // Push default configuration to the camera
    const Pds::Archon::Config& config = mDrv->config();
    mPixelCount = config.pixelcount();
    mLineCount = config.linecount();
    checkStatus(mDrv->set_number_of_pixels(mPixelCount), "Unable to set pixel count");
    checkStatus(mDrv->set_number_of_lines(mLineCount, batch), "Unable to set line count");
    checkStatus(mDrv->set_horizontal_binning(binX), "Unable to set horizontal binning");
    checkStatus(mDrv->set_vertical_binning(binY), "Unable to set vertical binning");
    checkStatus(mDrv->set_linescan_mode(false), "Unable to set linescan mode");
    checkStatus(mDrv->set_preframe_clear(preframeClear), "Unable to set preframe clear");
    checkStatus(mDrv->set_idle_clear(idleClear), "Unable to set idle clear");
    checkStatus(mDrv->set_preframe_skip(preframeSkip), "Unable to set preframe skip lines");
    checkStatus(mDrv->set_clock_at(clkat), "Unable to set clock AT");
    checkStatus(mDrv->set_clock_st(clkst), "Unable to set clock ST");
    checkStatus(mDrv->set_clock_stm1(clkstm1), "Unable to set clock STM1");
    checkStatus(mDrv->set_bias(biasChan, false, biasSetpoint), "Unable to set bias settings");
    checkStatus(mDrv->set_external_trigger(false), "Unable to set trigger");
    // set the frame polling period
    setupFramePoll(framePollingPeriod);

    // Get image configuration info
    sizeX = config.pixels_per_line();
    sizeY = config.linecount();
    sampleMode = config.samplemode();
    lineScanMode = config.linescan();
    bitsPerPixel = config.bytes_per_pixel() * 8; // convert to bits
    frameSize = config.frame_size();
    totalTaplines = config.taplines();
    activeTaplines = config.active_taplines();

    // Get the current temperature
    checkStatus(mDrv->fetch_status(), "Unable to fetch status info");
    const Pds::Archon::Status& status = mDrv->status();
    powerMode = status.power();
    printf("%s:%s: current power mode is %s\n", driverName, functionName, status.power_str());
    setIntegerParam(ArchonPowerMode, powerMode);
    double temperature = status.backplane_temp();
    printf("%s:%s: current backplane temperature is %f C\n", driverName, functionName, temperature);
    setDoubleParam(ADTemperature, temperature);

    // Get the bias voltage and current
    mBiasCache = false;
    mBiasChannelCache = biasChan;
    mBiasSetpointCache = biasSetpoint;
    checkStatus(mDrv->get_bias(biasChan, &biasVoltage, &biasCurrent),
                "Unable to read bias voltage and current");
    setDoubleParam(ArchonBiasVoltage, biasVoltage);
    setDoubleParam(ArchonBiasCurrent, biasCurrent);

    callParamCallbacks();
  } catch (const std::string &e) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: %s\n",
              driverName, functionName, e.c_str());
    return;
  }

  // Set some default parameters
  status = setStringParam(ADManufacturer, "STA");
  status |= setStringParam(ADSerialNumber, serialNumber.c_str());
  epicsSnprintf(tempString, sizeof(tempString), "Archon %s Rev %c",
                backplaneType < sizeofArray(ArchonCCD::BackplaneTypeEnums) ?
                BackplaneTypeEnums[backplaneType].name.c_str() :
                "Unknown",
                'A' + backplaneRev);
  status |= setStringParam(ADModel, tempString);
  status |= setStringParam(ADFirmwareVersion, firmwareVersion.c_str());
  epicsSnprintf(tempString, sizeof(tempString), "%d.%d.%d",
                Pds::Archon::ARCHON_VERSION,
                Pds::Archon::ARCHON_REVISION,
                Pds::Archon::ARCHON_MODIFICATION);
  status |= setStringParam(ADSDKVersion, tempString);
  epicsSnprintf(tempString, sizeof(tempString), "%d.%d.%d",
                DRIVER_VERSION, DRIVER_REVISION, DRIVER_MODIFICATION);
  status |= setStringParam(NDDriverVersion, tempString);
  status |= setIntegerParam(ADSizeX, sizeX);
  status |= setIntegerParam(ADSizeY, sizeY);
  status |= setIntegerParam(ADBinX, binX);
  status |= setIntegerParam(ADBinY, binY);
  status |= setIntegerParam(ADMinX, minX);
  status |= setIntegerParam(ADMinY, minY);
  status |= setIntegerParam(ADMaxSizeX, sizeX);
  status |= setIntegerParam(ADMaxSizeY, sizeY);
  status |= setIntegerParam(ADImageMode, ADImageSingle);
  status |= setIntegerParam(ADTriggerMode, ADTriggerInternal);
  status |= setDoubleParam (ADAcquireTime, 0.01);
  status |= setDoubleParam (ADAcquirePeriod, 5.0);
  status |= setDoubleParam (ArchonNonIntTime, 0.1);
  status |= setIntegerParam(ADNumImages, 1);
  status |= setIntegerParam(ADNumExposures, 1);
  status |= setIntegerParam(NDArraySizeX, sizeX);
  status |= setIntegerParam(NDArraySizeY, sizeY);
  status |= setIntegerParam(NDDataType, sampleMode ? NDUInt32 : NDUInt16);
  status |= setIntegerParam(NDArraySize, frameSize);
#ifdef NDBitsPerPixelString
  status |= setIntegerParam(NDBitsPerPixel, bitsPerPixel);
#endif
  status |= setIntegerParam(ADShutterMode, ADShutterModeNone);
  status |= setDoubleParam (ADShutterOpenDelay, 0.);
  status |= setDoubleParam (ADShutterCloseDelay, 0.);
  status |= setIntegerParam(ArchonPowerSwitch, powerMode==Pds::Archon::On);
  status |= setIntegerParam(ArchonReadOutMode, ARImage);
  status |= setIntegerParam(ArchonBackplaneType, backplaneType);
  status |= setIntegerParam(ArchonBackplaneRev, backplaneRev);
  status |= setIntegerParam(ArchonNumBatchFrames, batch);
  status |= setIntegerParam(ArchonLineScanMode, lineScanMode);
  status |= setIntegerParam(ArchonPreFrameClear, preframeClear);
  status |= setIntegerParam(ArchonIdleClear, idleClear);
  status |= setIntegerParam(ArchonPreFrameSkip, preframeSkip);
  status |= setIntegerParam(ArchonClockAt, clkat);
  status |= setIntegerParam(ArchonClockSt, clkst);
  status |= setIntegerParam(ArchonClockStm1, clkstm1);
  status |= setIntegerParam(ArchonBiasChan, biasChan);
  status |= setDoubleParam (ArchonBiasSetpoint, biasSetpoint);
  status |= setIntegerParam(ArchonBiasSwitch, false);
  status |= setDoubleParam (ArchonFramePollPeriod, framePollingPeriod);
  status |= setIntegerParam(ArchonTotalTaplines, totalTaplines);
  status |= setIntegerParam(ArchonActiveTaplines, activeTaplines);
  status |= setIntegerParam(ArchonPixelsPerTap, mPixelCount);

  callParamCallbacks();

  // send a signal to the status poller task
  epicsEventSignal(statusEvent);

  if (status) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: unable to set camera parameters\n",
              driverName, functionName);
    return;
  }

  // Define the polling periods for the status thread.
  mPollingPeriod = 0.2; // seconds
  mFastPollingPeriod = 0.05; // seconds
  // Define the polling period for the data task temperature reading
  mTempPollingPeriod = 0.5; // seconds

  mAcquiringData = 0;

  if (stackSize == 0) stackSize = epicsThreadGetStackSize(epicsThreadStackMedium);

  /* Create the thread that updates the detector status */
  status = (epicsThreadCreate("ArchonStatusTask",
                              epicsThreadPriorityMedium,
                              stackSize,
                              (EPICSTHREADFUNC)archonStatusTaskC,
                              this) == NULL);
  if (status) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: epicsThreadCreate failure for status task\n",
              driverName, functionName);
    return;
  }

  /* Create the thread that does data readout */
  status = (epicsThreadCreate("ArchonDataTask",
                              epicsThreadPriorityMedium,
                              stackSize,
                              (EPICSTHREADFUNC)archonDataTaskC,
                              this) == NULL);
  if (status) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: epicsThreadCreate failure for data task\n",
              driverName, functionName);
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

asynStatus ArchonCCD::readEnum(asynUser *pasynUser, char *strings[], int values[],
                               int severities[], size_t nElements, size_t *nIn)
{
  const char* name = NULL;
  int addr;
  int function = pasynUser->reason;
  size_t matched_size = 0;
  asynStatus status = asynSuccess;
  const ArchonEnumInfo *matched_enums = NULL;
  static const char *functionName = "readEnum";

  status = getAddress(pasynUser, &addr); if (status != asynSuccess) return status;

  getParamName(addr, function, &name);
  if (name) {
    asynPrint(pasynUser, ASYN_TRACEIO_DEVICE,
              "%s:%s: received enum read request for parameter: %s\n",
               driverName, functionName, name);
    /* Search the enum (if any) that goes with the requested parameter. */
    for (unsigned nEnum=0; nEnum<ArchonEnumsSize; nEnum++) {
      if (!strcmp(ArchonEnums[nEnum].name, name)) {
        matched_enums = ArchonEnums[nEnum].enums;
        matched_size = ArchonEnums[nEnum].size;
        break;
      }
    }

    if (matched_enums) {
      asynPrint(pasynUser, ASYN_TRACEIO_DEVICE,
              "%s:%s: enum found for parameter: %s\n",
               driverName, functionName, name);
      size_t i;
      for (i = 0; ((i < matched_size) && (i < nElements)); ++i) {
        if (strings[i]) free(strings[i]);
        strings[i] = epicsStrDup(matched_enums[i].name.c_str());
        values[i] = matched_enums[i].value;
        severities[i] = matched_enums[i].severity;
      }
      *nIn = i;
    } else {
      asynPrint(pasynUser, ASYN_TRACEIO_DEVICE,
              "%s:%s: no enum found for parameter: %s\n",
               driverName, functionName, name);
      *nIn = 0;
      status = asynError;
    }
  } else {
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: received enum read request for parameter with no name\n",
               driverName, functionName);
    *nIn = 0;
    status = asynError;
  }

  return status;
}

asynStatus ArchonCCD::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  int adstatus = 0;
  epicsInt32 oldValue;

  asynStatus status = asynSuccess;
  static const char *functionName = "writeInt32";

  // set param and save backup of old value
  getIntegerParam(function, &oldValue);
  status = setIntegerParam(function, value);

  if (function == ADAcquire) {
    getIntegerParam(ADStatus, &adstatus);
    if (value && (adstatus == ADStatusIdle)) {
      // Start the acqusition here, then send an event to the dataTask at the end of this function
      try {
        // Set up acquisition
        mAcquiringData = 1;
        status = setupAcquisition(true);
        if (status != asynSuccess) throw std::string("Setup acquisition failed");
        int adShutterMode;
        getIntegerParam(ADShutterMode, &adShutterMode);
        if (adShutterMode == ADShutterModeEPICS) {
          ADDriver::setShutter(ADShutterOpen);
        }
        int imageMode;
        int numImages = 0;
        getIntegerParam(ADImageMode, &imageMode);
        switch (imageMode) {
          case ADImageSingle:
            numImages = 1;
            break;
          case ADImageMultiple:
            getIntegerParam(ADNumImages, &numImages);
            break;
          case ADImageContinuous:
            numImages = 0;
            break;
        }
        // Start acquisition
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                  "%s:%s:, start_acquisition()\n",
                  driverName, functionName);
        checkStatus(mDrv->start_acquisition(numImages), "start_acquisition failed");
        // Reset the counters
        setIntegerParam(ADNumImagesCounter, 0);
        setIntegerParam(ADNumExposuresCounter, 0);
      } catch (const std::string &e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: %s\n",
                  driverName, functionName, e.c_str());
        status = asynError;
      }
    }
    if (!value && (adstatus != ADStatusIdle)) {
      try {
        mStopping = true;
        mDrv->timeout_waits();
        mDrvMutex->lock();
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                  "%s:%s:, clear_acquisition()\n",
                  driverName, functionName);
        checkStatus(mDrv->clear_acquisition(), "clear_acquisition failed");
        mAcquiringData = 0;
        mStopping = false;
        mDrv->timeout_waits(false);
        mDrvMutex->unlock();
      } catch (const std::string &e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: %s\n",
                  driverName, functionName, e.c_str());
        status = asynError;
      }
    }
  }
  else if ((function == ADNumExposures)     || (function == ADNumImages)            ||
           (function == ADImageMode)                                                ||
           (function == ADBinX)             || (function == ADBinY)                 ||
           (function == ADMinX)             || (function == ADMinY)                 ||
           (function == ADSizeX)            || (function == ADSizeY)                ||
           (function == ADReverseX)         || (function == ADReverseY)             ||
           (function == ADTriggerMode)      || (function == ArchonPowerSwitch)      ||
           (function == ArchonReadOutMode)  || (function == ArchonNumBatchFrames)   ||
           (function == ArchonLineScanMode) || (function == ArchonPreFrameClear)    ||
           (function == ArchonIdleClear)    || (function == ArchonPreFrameSkip)     ||
           (function == ArchonClockAt)      || (function == ArchonClockSt)          ||
           (function == ArchonClockStm1)    || (function == ArchonBiasChan)         ||
           (function == ArchonBiasSwitch)) {
    status = setupAcquisition();
    if (status != asynSuccess) setIntegerParam(function, oldValue);
  }
  else if (function == ADShutterControl) {
    setShutter(value);
  }
  else {
    status = ADDriver::writeInt32(pasynUser, value);
  }

  /* Do callbacks so higher layers see any changes */
  callParamCallbacks();

  /* Send a signal to the poller task which will make it do a poll, and switch to the fast poll rate */
  epicsEventSignal(statusEvent);

  if (mAcquiringData) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s:, Sending dataEvent to dataTask ...\n",
        driverName, functionName);
    // Also signal the data readout thread
    epicsEventSignal(dataEvent);
  }

  if (status) {
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, status=%d function=%d, value=%d\n",
              driverName, functionName, status, function, value);
  }
  else {
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, value=%d\n",
              driverName, functionName, function, value);
    // For a successful write, clear the error message.
    setStringParam(ArchonMessage, " ");
  }
  return status;
}

/** Called when asyn clients call pasynFloat64->write().
 * This function performs actions for some parameters.
 * For all parameters it sets the value in the parameter library and calls any registered callbacks.
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write. */
asynStatus ArchonCCD::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  static const char *functionName = "writeFloat64";

  /* Store the old value */
  epicsFloat64 oldValue;
  getDoubleParam(function, &oldValue);

  /* Set the parameter and readback in the parameter library.  */
  status = setDoubleParam(function, value);

  if (function == ADAcquireTime) {
    mAcquireTime = archonTimeConvert(value);
    status = setupAcquisition();
  }
  else if (function == ADAcquirePeriod) {
    mAcquirePeriod = archonTimeConvert(value);
    status = setupAcquisition();
  }
  else if (function == ArchonNonIntTime) {
    mNonIntTime = archonTimeConvert(value);
    status = setupAcquisition();
  }
  else if (function == ArchonBiasSetpoint) {
    status = setupAcquisition();
  }
  else if (function == ArchonFramePollPeriod) {
    status = setupFramePoll(value);
  }
  else {
    status = ADDriver::writeFloat64(pasynUser, value);
  }

  /* Do callbacks so higher layers see any changes */
  callParamCallbacks();
  if (status) {
    asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, status=%d function=%d, value=%f\n",
              driverName, functionName, status, function, value);
  }
  else {
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, value=%f\n",
              driverName, functionName, function, value);
    /* For a successful write, clear the error message. */
    setStringParam(ArchonMessage, " ");
  }
  return status;
}

asynStatus ArchonCCD::setupFramePoll(double period)
{
  asynStatus status=asynSuccess;
  static const char *functionName = "setupFramePoll";

  if (period < 0.0) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: frame polling period must be positive -> (%f)\n",
               driverName, functionName, period);
    status=asynError;
  } else {
    unsigned period_ms = (unsigned) (period*1e6);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s:, set_frame_poll_interval(%u)\n",
              driverName, functionName, period_ms);
    mDrv->set_frame_poll_interval(period_ms);
  }

  return status;
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

void ArchonCCD::statusTask(void)
{
  int biasChan;
  Pds::Archon::PowerMode powerMode;
  float temperature;
  float voltage;
  float current;
  unsigned int status = 0;
  double timeout = 0.0;
  unsigned int forcedFastPolls = 0;
  static const char *functionName = "statusTask";

  printf("%s:%s: Status thread started...\n", driverName, functionName);
  while(!mExiting) {

    // Read timeout for polling freq.
    this->lock();
    if (forcedFastPolls > 0) {
      timeout = mFastPollingPeriod;
      forcedFastPolls--;
    } else {
      timeout = mPollingPeriod;
    }
    this->unlock();

    if (timeout != 0.0) {
      status = epicsEventWaitWithTimeout(statusEvent, timeout);
    } else {
      status = epicsEventWait(statusEvent);
    }
    if (status == epicsEventWaitOK) {
      asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
        "%s:%s: Got status event\n",
        driverName, functionName);
      // We got an event, rather than a timeout.  This is because other software
      // knows that data has arrived, or device should have changed state (parameters changed, etc.).
      // Force a minimum number of fast polls, because the device status
      // might not have changed in the first few polls
      forcedFastPolls = 5;
    }

    if (mExiting) break;
    this->lock();

    try {
      // Only read these if we are not acquiring data
      if (!mAcquiringData) {
        // Fetch the status info from controller
        checkStatus(mDrv->fetch_status(), "Unable to fetch status info");
        const Pds::Archon::Status& detStatus = mDrv->status();
        // Read temperature of CCD
        temperature = detStatus.backplane_temp();
        setDoubleParam(ADTemperatureActual, temperature);
        // Read power status of CCD
        powerMode = detStatus.power();
        setIntegerParam(ArchonPowerMode, powerMode);
        // TODO Read module status info
        // Read bias voltage and current
        getIntegerParam(ArchonBiasChan, &biasChan);
        checkStatus(mDrv->get_bias(biasChan, &voltage, &current),
                    "Unable to get bias voltage and current");
        setDoubleParam(ArchonBiasVoltage, voltage);
        setDoubleParam(ArchonBiasCurrent, current);
      }

      // Read detector status (idle, acquiring, error, etc.)
      switch (mDrv->acquisition_mode()) {
        case Pds::Archon::Stopped:
          setIntegerParam(ADStatus, ADStatusIdle);
          setStringParam(ADStatusMessage, "IDLE. Waiting on instructions.");
          break;
        case Pds::Archon::Fixed:
        case Pds::Archon::Continuous:
          setIntegerParam(ADStatus, ADStatusAcquire);
          setStringParam(ADStatusMessage, "Data acquisition in progress.");
          break;
        default:
          setIntegerParam(ADStatus, ADStatusError);
          setStringParam(ADStatusMessage, "Unable to communicate with device.");
          break;
      }
    } catch (const std::string &e) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
        "%s:%s: %s\n",
        driverName, functionName, e.c_str());
      setStringParam(ArchonMessage, e.c_str());
    }

    /* Call the callbacks to update any changes */
    callParamCallbacks();
    this->unlock();

  } // End of loop
  asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
      "%s:%s: Status thread exiting ...\n",
      driverName, functionName);

  mExited++;
}

asynStatus ArchonCCD::setupAcquisition(bool commit)
{
  int numExposures;
  int numImages;
  int imageMode;
  int triggerMode;
  int powerSwitch;
  int powerMode;
  int readOutMode;
  int numBatchFrames;
  int preframeClear;
  int idleClear;
  int preframeSkip;
  int clockAt;
  int clockSt;
  int clockStm1;
  int biasChan;
  int biasSwitch;
  int binX, binY, minX, minY, sizeX, sizeY, reverseX, reverseY, maxSizeX, maxSizeY;
  unsigned sampleMode;
  unsigned lineScanMode;
  unsigned frameSize;
  unsigned bitsPerPixel;
  double biasSetpoint;
  static const char *functionName = "setupAcquisition";

  if (!mInitOK) {
    return asynDisabled;
  }

  // Make sure current image/binning params are sane
  getIntegerParam(ArchonReadOutMode, &readOutMode);
  getIntegerParam(ADImageMode, &imageMode);
  getIntegerParam(ADNumExposures, &numExposures);
  if (numExposures <= 0) {
    numExposures = 1;
    setIntegerParam(ADNumExposures, numExposures);
  }
  getIntegerParam(ADNumImages, &numImages);
  if (numImages <= 0) {
    numImages = 1;
    setIntegerParam(ADNumImages, numImages);
  }
  getIntegerParam(ADBinX, &binX);
  if (binX <= 0) {
    binX = 1;
    setIntegerParam(ADBinX, binX);
  }
  getIntegerParam(ADBinY, &binY);
  if (binY <= 0) {
    binY = 1;
    setIntegerParam(ADBinY, binY);
  }
  getIntegerParam(ADMinX, &minX);
  getIntegerParam(ADMinY, &minY);
  getIntegerParam(ADSizeX, &sizeX);
  getIntegerParam(ADSizeY, &sizeY);
  getIntegerParam(ADReverseX, &reverseX);
  getIntegerParam(ADReverseY, &reverseY);
  getIntegerParam(ADMaxSizeX, &maxSizeX);
  getIntegerParam(ADMaxSizeY, &maxSizeY);
  if (readOutMode == ARFullVerticalBinning) {
    // Set maximum binning but do not update parameter, this preserves ADBinY
    // when going back to Image readout mode.
    minY = 0;
    sizeY = maxSizeY;
    binY = maxSizeY;
  }
  if (minX > (maxSizeX - binX)) {
    minX = maxSizeX - binX;
    setIntegerParam(ADMinX, minX);
  }
  if (minY > (maxSizeY - binY)) {
    minY = maxSizeY - binY;
    setIntegerParam(ADMinY, minY);
  }
  if ((minX + sizeX) > maxSizeX) {
    sizeX = maxSizeX - minX;
    setIntegerParam(ADSizeX, sizeX);
  }
  if ((minY + sizeY) > maxSizeY) {
    sizeY = maxSizeY - minY;
    setIntegerParam(ADSizeY, sizeY);
  }

  if (!commit) {
    return asynSuccess;
  }

  getIntegerParam(ADTriggerMode, &triggerMode);

  getIntegerParam(ArchonPowerMode, &powerMode);
  getIntegerParam(ArchonPowerSwitch, &powerSwitch);

  getIntegerParam(ArchonNumBatchFrames, &numBatchFrames);
  getIntegerParam(ArchonPreFrameClear, &preframeClear);
  getIntegerParam(ArchonIdleClear, &idleClear);
  getIntegerParam(ArchonPreFrameSkip, &preframeSkip);
  getIntegerParam(ArchonClockAt, &clockAt);
  getIntegerParam(ArchonClockSt, &clockSt);
  getIntegerParam(ArchonClockStm1, &clockStm1);

  getIntegerParam(ArchonBiasChan, &biasChan);
  getIntegerParam(ArchonBiasSwitch, &biasSwitch);
  getDoubleParam (ArchonBiasSetpoint, &biasSetpoint);

  try {
    // only setup the bias if it has changed since this is slow and requires power cycle
    if ((biasChan != mBiasChannelCache) ||
        (biasSwitch != mBiasCache) ||
        (fabs(biasSetpoint - mBiasSetpointCache) > 0.05)) {
      // need to power off to apply bias settings
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s:, power_off()\n",
                driverName, functionName);
      checkStatus(mDrv->power_off(), "Unable to power off ccd");
      powerMode = Pds::Archon::Off;

      // Set the detector bias
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s:, set_bias(%s, %s, %f)\n",
                driverName, functionName,
                BiasChannelEnums[biasChan].name.c_str(),
                biasSwitch ?  "true" : "false",
                biasSetpoint);
      checkStatus(mDrv->set_bias(biasChan, biasSwitch, biasSetpoint), "Unable to set bias settings");
      mBiasCache = biasSwitch;
      mBiasChannelCache = biasChan;
      mBiasSetpointCache = biasSetpoint;
    }
    if (powerSwitch && powerMode < Pds::Archon::Intermediate) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s:, power_on()\n",
                driverName, functionName);
      checkStatus(mDrv->power_on(), "Unable to power on ccd");
    } else if (!powerSwitch && powerMode > Pds::Archon::Intermediate) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s:, power_off()\n",
                driverName, functionName);
      checkStatus(mDrv->power_off(), "Unable to power off ccd");
    }
    // Set the binning parameters and number of vertical lines
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s:, set_vertical_binning(%d)\n",
              driverName, functionName, binY);
    checkStatus(mDrv->set_vertical_binning(binY), "unable to set vertical binning");
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s:, set_horizontal_binning(%d)\n",
              driverName, functionName, binX);
    checkStatus(mDrv->set_horizontal_binning(binX), "unable to set horizontal binning");
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s:, set_number_of_lines(%d, %d)\n",
              driverName, functionName, sizeY/binY, numBatchFrames);
    checkStatus(mDrv->set_number_of_lines(sizeY/binY, (unsigned) numBatchFrames),
                "unable to set number of lines");
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s:, set_number_of_pixels(%d)\n",
              driverName, functionName, mPixelCount/binX);
    checkStatus(mDrv->set_number_of_pixels(mPixelCount/binX), "unable to set number of pixels");
    // Set clearing parameters
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s:, set_preframe_clear(%d)\n",
              driverName, functionName, preframeClear);
    checkStatus(mDrv->set_preframe_clear(preframeClear), "unable to set preframe clear");
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s:, set_idle_clear(%d)\n",
              driverName, functionName, idleClear);
    checkStatus(mDrv->set_idle_clear(idleClear), "unable to set idle clear");
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s:, set_preframe_skip(%d)\n",
              driverName, functionName, preframeSkip);
    checkStatus(mDrv->set_preframe_skip(preframeSkip), "unable to set preframe skip");
    // Set exposure time parameters
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s:, set_integration_time(%d)\n",
              driverName, functionName, mAcquireTime);
    checkStatus(mDrv->set_integration_time(mAcquireTime), "unable to set integration time");
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s:, set_non_integration_time(%d)\n",
              driverName, functionName, mNonIntTime);
    checkStatus(mDrv->set_non_integration_time(mNonIntTime), "unable to set non-integration time");
    // Set the external trigger
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
              "%s:%s:, set_external_trigger(%s)\n",
              driverName, functionName, triggerMode ?  "true" : "false");
    checkStatus(mDrv->set_external_trigger(triggerMode), "Unable to set external trigger setting");

    // Get image configuration info and apply it
    const Pds::Archon::Config& config = mDrv->config();
    sizeX = config.pixels_per_line();
    sizeY = config.linecount();
    lineScanMode = config.linescan();
    sampleMode = config.samplemode();
    frameSize = config.frame_size();
    bitsPerPixel = config.bytes_per_pixel() * 8;

    if (frameSize != mFrameBufferSize) {
      if (mFrameBuffer) {
        free(mFrameBuffer);
      }

      mFrameBuffer = malloc(frameSize);
      mFrameBufferSize = frameSize;
    }

    // adjust the sizeY and framesize if  in batch mode before setting parameters
    if (numBatchFrames > 0) {
      sizeY /= numBatchFrames;
      frameSize /= numBatchFrames;
    }

    setIntegerParam(ArchonLineScanMode, lineScanMode);
    setIntegerParam(NDArraySizeX, sizeX);
    setIntegerParam(NDArraySizeY, sizeY);
    setIntegerParam(NDDataType, sampleMode ? NDUInt32 : NDUInt16);
    setIntegerParam(NDArraySize, frameSize);
#ifdef NDBitsPerPixelString
    setIntegerParam(NDBitsPerPixel, bitsPerPixel);
#endif
  } catch (const std::string &e) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: %s\n",
              driverName, functionName, e.c_str());
    return asynError;
  }
  return asynSuccess;
}

bool ArchonCCD::waitFrame(void *frameBuffer, Pds::Archon::FrameMetaData *frameMeta)
{
  bool ret;

  // release the IOC lock
  this->unlock();
  // acquire the lock on the detector
  mDrvMutex->lock();
  // run the actual frame command
  ret = mDrv->wait_frame(mFrameBuffer, frameMeta);
  // release the lock on the detector
  mDrvMutex->unlock();
  // acquire the IOC lock
  this->lock();

  return ret;
}

void ArchonCCD::dataTask(void)
{
  epicsUInt32 status = 0;
  Pds::Archon::AcqMode acquireStatus;
  int bitsPerPixel = 16;
  char *errorString = NULL;
  int acquiring = 0;
  epicsInt32 numImagesCounter;
  epicsInt32 numExposuresCounter;
  epicsInt32 imageCounter;
  epicsInt32 arrayCallbacks;
  epicsInt32 sizeX, sizeY;
  int adShutterMode;
  NDDataType_t dataType;
  int itemp;
//TODO  at_32 firstImage, lastImage;
//TODO  at_32 validFirst, validLast;
  size_t dims[2];
  int nDims = 2;
  int i;
  epicsTimeStamp startTime;
  epicsTimeStamp currentTempTime;
  epicsTimeStamp lastTempTime;
  NDArray *pArray;
  Pds::Archon::FrameMetaData frameMeta;
  int autoSave;
  int readOutMode;
  float temperature;
  static const char *functionName = "dataTask";

  printf("%s:%s: Data thread started...\n", driverName, functionName);

  this->lock();

  while(!mExiting) {

    errorString = NULL;

    // Wait for event from main thread to signal that data acquisition has started.
    this->unlock();
    status = epicsEventWait(dataEvent);
    if (mExiting)
        break;
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
      "%s:%s:, got data event\n",
      driverName, functionName);
    this->lock();

    // Sanity check that main thread thinks we are acquiring data
    if (mAcquiringData) {
      // Read some parameters
      getIntegerParam(ADShutterMode, &adShutterMode);
      getIntegerParam(ArchonReadOutMode, &readOutMode);
      getIntegerParam(NDDataType, &itemp); dataType = (NDDataType_t)itemp;
      getIntegerParam(NDAutoSave, &autoSave);
      getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
      getIntegerParam(NDArraySizeX, &sizeX);
      getIntegerParam(NDArraySizeY, &sizeY);
      // Set acquiring to 1
      acquiring = 1;
    } else {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
        "%s:%s:, Data thread is running but main thread thinks we are not acquiring.\n",
        driverName, functionName);
      // Set acquiring to 0
      acquiring = 0;
    }

    while ((acquiring) && (!mExiting) && (!mStopping)) {
      try {
        acquireStatus = mDrv->acquisition_mode();
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                  "%s:%s:, acquisition_mode returned %d\n",
                  driverName, functionName, acquireStatus);
        if (acquireStatus == Pds::Archon::Stopped) break;
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                  "%s:%s:, wait_frame().\n",
                  driverName, functionName);
        checkStatus(waitFrame(mFrameBuffer, &frameMeta), "failure in wait_frame");
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                  "%s:%s:, wait_frame() has returned.\n",
                  driverName, functionName);
        getIntegerParam(ADNumExposuresCounter, &numExposuresCounter);
        numExposuresCounter++;
        setIntegerParam(ADNumExposuresCounter, numExposuresCounter);
        callParamCallbacks();

      } catch (const std::string &e) {
          if (!mExiting)
          {
              asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s:%s: %s\n",
                  driverName, functionName, e.c_str());
              errorString = const_cast<char *>(e.c_str());
              setStringParam(ArchonMessage, errorString);
          }
      }
    }

    // Close the shutter if we are controlling it
    if (adShutterMode == ADShutterModeEPICS) {
      ADDriver::setShutter(ADShutterClosed);
    }

    // Now clear main thread flag
    mAcquiringData = 0;
    setIntegerParam(ADAcquire, 0);
    //setIntegerParam(ADStatus, 0); //Dont set this as the status thread sets it.

    /* Call the callbacks to update any changes */
    callParamCallbacks();
  } // End of loop
  mExited++;
  this->unlock();
}

#undef MAX_ENUM_STRING_SIZE
#undef sizeofArray

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
