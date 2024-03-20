/**
 * Area Detector driver for the Archon CCD.
 *
 * @author Daniel Damiani
 * @date Aug 2022
 */

#ifndef ARCHONCCD_H
#define ARCHONCCD_H

#include "ADDriver.h"

#include <alarm.h>

#define ArchonMessageString           "ARCHON_MESSAGE"
#define ArchonShutterMessageString    "ARCHON_SHUTTER_MESSAGE"
#define ArchonPwrStatusMessageString  "ARCHON_PWR_STAT"
#define ArchonHtrStatusMessageString  "ARCHON_HTR_STAT"
#define ArchonSenStatusMessageString  "ARCHON_SEN_STAT"
#define ArchonBackplaneTypeString     "ARCHON_BACKPLANE_TYPE"
#define ArchonBackplaneRevString      "ARCHON_BACKPLANE_REV"
#define ArchonModuleTypeString        "ARCHON_MODULE_TYPE"
#define ArchonModuleRevString         "ARCHON_MODULE_REV"
#define ArchonModuleFirmwareString    "ARCHON_MODULE_FIRMWARE"
#define ArchonModuleIdString          "ARCHON_MODULE_ID"
#define ArchonModuleInfoString        "ARCHON_MODULE_INFO"
#define ArchonModuleTempString        "ARCHON_MODULE_TEMP"
#define ArchonOverheatString          "ARCHON_OVERHEAT"
#define ArchonPowerStatusString       "ARCHON_POWER_STATUS"
#define ArchonPowerModeString         "ARCHON_POWER_MODE"
#define ArchonPowerSwitchString       "ARCHON_POWER_SWITCH"
#define ArchonReadOutModeString       "ARCHON_READOUT_MODE"
#define ArchonBatchDelayString        "ARCHON_BATCH_DELAY"
#define ArchonMinBatchPeriodString    "ARCHON_MIN_BATCH_PERIOD"
#define ArchonNumBatchFramesString    "ARCHON_NUM_BATCH_FRAMES"
#define ArchonLineScanModeString      "ARCHON_LINESCAN_MODE"
#define ArchonPreFrameClearString     "ARCHON_PREFRAME_CLEAR"
#define ArchonIdleClearString         "ARCHON_IDLE_CLEAR"
#define ArchonPreFrameSkipString      "ARCHON_PREFRAME_SKIP"
#define ArchonNonIntTimeString        "ARCHON_NONINT_TIME"
#define ArchonClockAtString           "ARCHON_CLOCK_AT"
#define ArchonClockStString           "ARCHON_CLOCK_ST"
#define ArchonClockStm1String         "ARCHON_CLOCK_STM1"
#define ArchonClockCtString           "ARCHON_CLOCK_CT"
#define ArchonNumDummyPixelsString    "ARCHON_NUM_DUMMY_PIXELS"
#define ArchonClearTimeString         "ARCHON_CLEAR_TIME"
#define ArchonReadOutTimeString       "ARCHON_READOUT_TIME"
#define ArchonBiasChanString          "ARCHON_BIAS_CHAN"
#define ArchonBiasSetpointString      "ARCHON_BIAS_SETPOINT"
#define ArchonBiasSwitchString        "ARCHON_BIAS_SWITCH"
#define ArchonBiasVoltageString       "ARCHON_BIAS_VOLTAGE"
#define ArchonBiasCurrentString       "ARCHON_BIAS_CURRENT"
#define ArchonFramePollPeriodString   "ARCHON_FRAME_POLL_PERIOD"
#define ArchonTotalTaplinesString     "ARCHON_TOTAL_TAPLINES"
#define ArchonActiveTaplinesString    "ARCHON_ACTIVE_TAPLINES"
#define ArchonPixelsPerTapString      "ARCHON_PIXELS_PER_TAP"
#define ArchonShutterModeString       "ARCHON_SHUTTER_MODE"
#define ArchonShutterPolarityString   "ARCHON_SHUTTER_POLARITY"
#define ArchonHeaterLabelString       "ARCHON_HEATER_LABEL"
#define ArchonHeaterEnableString      "ARCHON_HEATER_ENABLE"
#define ArchonHeaterForceString       "ARCHON_HEATER_FORCE"
#define ArchonHeaterForceLevelString  "ARCHON_HEATER_FORCELEVEL"
#define ArchonHeaterLimitString       "ARCHON_HEATER_LIMIT"
#define ArchonHeaterTargetString      "ARCHON_HEATER_TARGET"
#define ArchonHeaterSensorString      "ARCHON_HEATER_SENSOR"
#define ArchonHeaterPTermString       "ARCHON_HEATER_PTERM"
#define ArchonHeaterITermString       "ARCHON_HEATER_ITERM"
#define ArchonHeaterDTermString       "ARCHON_HEATER_DTERM"
#define ArchonHeaterITermLimitString  "ARCHON_HEATER_ITERMLIMIT"
#define ArchonHeaterRampString        "ARCHON_HEATER_RAMP"
#define ArchonHeaterRampRateString    "ARCHON_HEATER_RAMPRATE"
#define ArchonHeaterUpdateTimeString  "ARCHON_HEATER_UPDATETIME"
#define ArchonHeaterOutputString      "ARCHON_HEATER_OUTPUT"
#define ArchonHeaterPTermReadString   "ARCHON_HEATER_PTERM_READ"
#define ArchonHeaterITermReadString   "ARCHON_HEATER_ITERM_READ"
#define ArchonHeaterDTermReadString   "ARCHON_HEATER_DTERM_READ"
#define ArchonSensorLabelString       "ARCHON_SENSOR_LABEL"
#define ArchonSensorTypeString        "ARCHON_SENSOR_TYPE"
#define ArchonSensorCurrentString     "ARCHON_SENSOR_CURRENT"
#define ArchonSensorLowerLimitString  "ARCHON_SENSOR_LOWERLIMIT"
#define ArchonSensorUpperLimitString  "ARCHON_SENSOR_UPPERLIMIT"
#define ArchonSensorFilterString      "ARCHON_SENSOR_FILTER"
#define ArchonSensorTempString        "ARCHON_SENSOR_TEMP"
#define ArchonConfigFileString        "ARCHON_CONFIG_FILE"

#define ArchonMaxHeaters 2
#define ArchonMaxSensors 3
#define ArchonMaxModules 12

/**
 * Forward declare the Driver class from the LCLS-I DAQ library.
 */
namespace Pds {
  namespace Archon {
    class Driver;
    class FrameMetaData;
  }
}

/**
 * Driver for Archon CCD cameras using the LCLS-I DAQ library; inherits from ADDriver class in ADCore.
 *
 */
class ArchonCCD : public ADDriver {
  public:
    ArchonCCD(const char *portName, const char *filePath, const char *cameraAddr, int cameraPort,
              int maxBuffers, size_t maxMemory, int priority, int stackSize);
    virtual ~ArchonCCD();

    /* These are the methods that we override from ADDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus writeOctet(asynUser *pasynUser, const char *value,
                                  size_t nChars, size_t *nActual);
    //virtual void report(FILE *fp, int details);
    virtual asynStatus readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[],
                                size_t nElements, size_t *nIn);

    // Should be private, but are called from C so must be public
    void statusTask(void);
    void dataTask(void);

  protected:
    // enum information
    typedef struct {
      const std::string name;
      int value;
      epicsAlarmSeverity severity;
    } ArchonEnumInfo;
    typedef struct {
      const ArchonEnumInfo *enums;
      size_t size;
      const char *name;
    } ArchonEnumSet;
    static const ArchonEnumInfo OnOffEnums[];
    static const ArchonEnumInfo EnableDisableEnums[];
    static const ArchonEnumInfo BackplaneTypeEnums[];
    static const ArchonEnumInfo OverheatEnums[];
    static const ArchonEnumInfo PowerStatusEnums[];
    static const ArchonEnumInfo PowerModeEnums[];
    static const ArchonEnumInfo ReadOutModeEnums[];
    static const ArchonEnumInfo BiasChannelEnums[];
    static const ArchonEnumInfo ModuleTypeEnums[];
    static const ArchonEnumInfo ShutterModeEnums[];
    static const ArchonEnumInfo ShutterPolarityEnums[];
    static const ArchonEnumInfo SensorNameEnums[];
    static const ArchonEnumInfo SensorTypeEnums[];
    static const ArchonEnumInfo SensorFilterEnums[];
    static const ArchonEnumSet ArchonEnums[];
    static const size_t ArchonEnumsSize;
    static const ArchonEnumSet ArchonEnumsSpecial[];
    static const size_t ArchonEnumsSpecialSize;

    // parameters
    int ArchonMessage;
    int ArchonShutterMessage;
    int ArchonPwrStatusMessage;
    int ArchonHtrStatusMessage;
    int ArchonSenStatusMessage;
    int ArchonBackplaneType;
    int ArchonBackplaneRev;
    int ArchonModuleType[ArchonMaxModules];
    int ArchonModuleRev[ArchonMaxModules];
    int ArchonModuleFirmware[ArchonMaxModules];
    int ArchonModuleId[ArchonMaxModules];
    int ArchonModuleInfo[ArchonMaxModules];
    int ArchonModuleTemp[ArchonMaxModules];
    int ArchonOverheat;
    int ArchonPowerStatus;
    int ArchonPowerMode;
    int ArchonPowerSwitch;
    int ArchonReadOutMode;
    int ArchonBatchDelay;
    int ArchonMinBatchPeriod;
    int ArchonNumBatchFrames;
    int ArchonLineScanMode;
    int ArchonPreFrameClear;
    int ArchonIdleClear;
    int ArchonPreFrameSkip;
    int ArchonNonIntTime;
    int ArchonClockAt;
    int ArchonClockSt;
    int ArchonClockStm1;
    int ArchonClockCt;
    int ArchonNumDummyPixels;
    int ArchonClearTime;
    int ArchonReadOutTime;
    int ArchonBiasChan;
    int ArchonBiasSetpoint;
    int ArchonBiasSwitch;
    int ArchonBiasVoltage;
    int ArchonBiasCurrent;
    int ArchonFramePollPeriod;
    int ArchonTotalTaplines;
    int ArchonActiveTaplines;
    int ArchonPixelsPerTap;
    int ArchonShutterMode;
    int ArchonShutterPolarity;
    int ArchonHeaterLabel[ArchonMaxHeaters];
    int ArchonHeaterEnable[ArchonMaxHeaters];
    int ArchonHeaterForce[ArchonMaxHeaters];
    int ArchonHeaterForceLevel[ArchonMaxHeaters];
    int ArchonHeaterLimit[ArchonMaxHeaters];
    int ArchonHeaterTarget[ArchonMaxHeaters];
    int ArchonHeaterSensor[ArchonMaxHeaters];
    int ArchonHeaterPTerm[ArchonMaxHeaters];
    int ArchonHeaterITerm[ArchonMaxHeaters];
    int ArchonHeaterDTerm[ArchonMaxHeaters];
    int ArchonHeaterITermLimit[ArchonMaxHeaters];
    int ArchonHeaterRamp[ArchonMaxHeaters];
    int ArchonHeaterRampRate[ArchonMaxHeaters];
    int ArchonHeaterOutput[ArchonMaxHeaters];
    int ArchonHeaterPTermRead[ArchonMaxHeaters];
    int ArchonHeaterITermRead[ArchonMaxHeaters];
    int ArchonHeaterDTermRead[ArchonMaxHeaters];
    int ArchonSensorLabel[ArchonMaxSensors];
    int ArchonSensorType[ArchonMaxSensors];
    int ArchonSensorCurrent[ArchonMaxSensors];
    int ArchonSensorLowerLimit[ArchonMaxSensors];
    int ArchonSensorUpperLimit[ArchonMaxSensors];
    int ArchonSensorFilter[ArchonMaxSensors];
    int ArchonSensorTemp[ArchonMaxSensors];
    int ArchonHeaterUpdateTime;
    int ArchonConfigFile;
#define FIRST_ARCHON_PARAM ArchonMessage
#define LAST_ARCHON_PARAM ArchonConfigFile

  private:

    bool checkStatus(bool status, const char *message);
    asynStatus setupAcquisition(bool commit=false);
    asynStatus setupShutter(int command);
    asynStatus setupHeater(int heater);
    asynStatus setupSensor(int sensor);
    asynStatus setupPowerAndBias();
    asynStatus setupFramePoll(double period);
    asynStatus setupFileWrite(bool trigger=false);
    epicsUInt64 calcClearTime(epicsUInt64 at,
                              epicsUInt64 st,
                              epicsUInt64 stm1,
                              epicsUInt64 skips,
                              epicsUInt64 sweeps);
    epicsUInt64 calcReadOutTime(epicsUInt64 at,
                                epicsUInt64 st,
                                epicsUInt64 stm1,
                                epicsUInt64 sizey,
                                epicsUInt64 binx,
                                epicsUInt64 biny);
    bool waitFrame(void *frameBuffer, Pds::Archon::FrameMetaData *frameMeta);
    bool saveDataFrame(int frameNumber, bool append=false);

    static unsigned archonTimeConvert(double time_in_sec);
    static epicsUInt64 archonClockConvert(double time_in_sec);

    /**
     * List of boolean states (for bi/bo records)
     */
    static const epicsInt32 ABFalse;
    static const epicsInt32 ABTrue;

    /**
     * List of controller backplane types
     */
    static const epicsInt32 ABNone;
    static const epicsInt32 ABX12;
    static const epicsInt32 ABX16;
    static const epicsInt32 ABUnknown;

    /**
     * List of detector power modes.
     */
    static const epicsInt32 APUnknown;
    static const epicsInt32 APNotConfigured;
    static const epicsInt32 APOff;
    static const epicsInt32 APIntermediate;
    static const epicsInt32 APOn;
    static const epicsInt32 APStandby;

    /**
     * List of detector readout modes.
     */
    static const epicsInt32 ARImage;
    static const epicsInt32 ARFullVerticalBinning;

    /**
     * List of detector bias channels.
     */
    static const epicsInt32 ABNV4;
    static const epicsInt32 ABNV3;
    static const epicsInt32 ABNV2;
    static const epicsInt32 ABNV1;
    static const epicsInt32 ABPV1;
    static const epicsInt32 ABPV2;
    static const epicsInt32 ABPV3;
    static const epicsInt32 ABPV4;

    /**
     * List of detector module types
     */
    static const epicsInt32 AMNone;
    static const epicsInt32 AMDriver;
    static const epicsInt32 AMAD;
    static const epicsInt32 AMLVBias;
    static const epicsInt32 AMHVBias;
    static const epicsInt32 AMHeater;
    static const epicsInt32 AMUnknown;
    static const epicsInt32 AMHS;
    static const epicsInt32 AMHVXBias;
    static const epicsInt32 AMLVXBias;
    static const epicsInt32 AMLVDS;
    static const epicsInt32 AMHeaterX;
    static const epicsInt32 AMXVBias;
    static const epicsInt32 AMADF;
    static const epicsInt32 AMADX;
    static const epicsInt32 AMADLN;

    /**
     * List of detector shutter modes.
     */
    static const epicsInt32 AShutterFullyAuto;
    static const epicsInt32 AShutterAlwaysOpen;
    static const epicsInt32 AShutterAlwaysClosed;

    /**
     * List of file formats
     */
    static const epicsInt32 AFRAW;

    /**
     * List of sensor names.
     */
    static const epicsInt32 ASA;
    static const epicsInt32 ASB;
    static const epicsInt32 ASC;

    /**
     * List of sensor types.
     */
    static const epicsInt32 ASDT670;
    static const epicsInt32 ASDT470;
    static const epicsInt32 ASRTD100;
    static const epicsInt32 ASRTD400;
    static const epicsInt32 ASRTD1000;
    static const epicsInt32 ASRTD2000;

    /**
     * List of filters
     */
    static const epicsInt32 ASDisabled;
    static const epicsInt32 ASFilter2;
    static const epicsInt32 ASFilter4;
    static const epicsInt32 ASFilter8;
    static const epicsInt32 ASFilter16;
    static const epicsInt32 ASFilter32;
    static const epicsInt32 ASFilter64;
    static const epicsInt32 ASFilter128;
    static const epicsInt32 ASFilter256;

    // seconds per timing core clock
    static const epicsFloat64 SECS_PER_CLOCK;
    static const epicsUInt64 CLOCK_PER_MSEC;

    epicsEventId statusEvent;
    epicsEventId dataEvent;
    double mPollingPeriod;
    double mFastPollingPeriod;
    double mTempPollingPeriod;
    unsigned int mAcquiringData;
    Pds::Archon::Driver *mDrv;
    epicsMutex *mDrvMutex;
    int mCaptureBufferSize;
    NDArray **mCaptureBufferArrays;
    Pds::Archon::FrameMetaData *mCaptureBufferMetaData;
    char mFullFileName[MAX_FILENAME_LEN];
    bool mStopping;
    bool mExiting;
    int mExited;
    void* mFrameBuffer;
    size_t mFrameBufferSize;

    unsigned mAcquireTime;
    unsigned mNonIntTime;
    unsigned mAcquirePeriod;

    // default sensor size
    unsigned mPixelCount;
    unsigned mLineCount;
    // readout time constants
    unsigned mDummyPixelCount;
    unsigned mClockCt;
    // clear and readout time in clock ticks
    epicsUInt64 mClearTime;
    epicsUInt64 mReadOutTime;
    epicsUInt64 mMinBatchPeriod;

    // last set bias value cache
    bool mBiasCache;
    int mBiasChannelCache;
    float mBiasSetpointCache;
    Pds::Archon::HeaterConfig mHeaterCache[ArchonMaxHeaters];
    Pds::Archon::SensorConfig mSensorCache[ArchonMaxSensors];

    bool mInitOK;
};

#endif //ARCHONCCD_H 
