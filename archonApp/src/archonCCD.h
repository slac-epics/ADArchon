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

#define ArchonMessageString         "ARCHON_MESSAGE"
#define ArchonBackplaneTypeString   "ARCHON_BACKPLANE_TYPE"
#define ArchonBackplaneRevString    "ARCHON_BACKPLANE_REV"
#define ArchonPowerModeString       "ARCHON_POWER_MODE"
#define ArchonPowerSwitchString     "ARCHON_POWER_SWITCH"
#define ArchonReadOutModeString     "ARCHON_READOUT_MODE"
#define ArchonNumBatchFramesString  "ARCHON_NUM_BATCH_FRAMES"
#define ArchonLineScanModeString    "ARCHON_LINESCAN_MODE"
#define ArchonPreFrameClearString   "ARCHON_PREFRAME_CLEAR"
#define ArchonIdleClearString       "ARCHON_IDLE_CLEAR"
#define ArchonPreFrameSkipString    "ARCHON_PREFRAME_SKIP"
#define ArchonNonIntTimeString      "ARCHON_NONINT_TIME"
#define ArchonClockAtString         "ARCHON_CLOCK_AT"
#define ArchonClockStString         "ARCHON_CLOCK_ST"
#define ArchonClockStm1String       "ARCHON_CLOCK_STM1"
#define ArchonBiasChanString        "ARCHON_BIAS_CHAN"
#define ArchonBiasSetpointString    "ARCHON_BIAS_SETPOINT"
#define ArchonBiasSwitchString      "ARCHON_BIAS_SWITCH"
#define ArchonBiasVoltageString     "ARCHON_BIAS_VOLTAGE"
#define ArchonBiasCurrentString     "ARCHON_BIAS_CURRENT"
#define ArchonFramePollPeriodString "ARCHON_FRAME_POLL_PERIOD"
#define ArchonTotalTaplinesString   "ARCHON_TOTAL_TAPLINES"
#define ArchonActiveTaplinesString  "ARCHON_ACTIVE_TAPLINES"
#define ArchonPixelsPerTapString    "ARCHON_PIXELS_PER_TAP"
#define ArchonConfigFileString      "ARCHON_CONFIG_FILE"

/**
 * Forward declare the Driver class from the LCLS-I DAQ library.
 */
namespace Pds {
  namespace Archon {
    class Driver;
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
    static const ArchonEnumInfo BackplaneTypeEnums[];
    static const ArchonEnumInfo PowerModeEnums[];
    static const ArchonEnumInfo ReadOutModeEnums[];
    static const ArchonEnumInfo BiasChannelEnums[];
    static const ArchonEnumSet ArchonEnums[];
    static const size_t ArchonEnumsSize;

    // parameters
    int ArchonMessage;
    int ArchonBackplaneType;
    int ArchonBackplaneRev;
    int ArchonPowerMode;
    int ArchonPowerSwitch;
    int ArchonReadOutMode;
    int ArchonNumBatchFrames;
    int ArchonLineScanMode;
    int ArchonPreFrameClear;
    int ArchonIdleClear;
    int ArchonPreFrameSkip;
    int ArchonNonIntTime;
    int ArchonClockAt;
    int ArchonClockSt;
    int ArchonClockStm1;
    int ArchonBiasChan;
    int ArchonBiasSetpoint;
    int ArchonBiasSwitch;
    int ArchonBiasVoltage;
    int ArchonBiasCurrent;
    int ArchonFramePollPeriod;
    int ArchonTotalTaplines;
    int ArchonActiveTaplines;
    int ArchonPixelsPerTap;
    int ArchonConfigFile;
#define FIRST_ARCHON_PARAM ArchonMessage
#define LAST_ARCHON_PARAM ArchonConfigFile

  private:

    bool checkStatus(bool status, const char *message);
    asynStatus setupAcquisition(bool commit=false);
    asynStatus setupFramePoll(double period);
    bool waitFrame(void *frameBuffer, Pds::Archon::FrameMetaData *frameMeta);
    //void saveDataFrame(int frameNumber);

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

    epicsEventId statusEvent;
    epicsEventId dataEvent;
    double mPollingPeriod;
    double mFastPollingPeriod;
    double mTempPollingPeriod;
    unsigned int mAcquiringData;
    Pds::Archon::Driver *mDrv;
    epicsMutex *mDrvMutex;
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

    // last set bias value cache
    bool mBiasCache;
    int mBiasChannelCache;
    float mBiasSetpointCache;

    bool mInitOK;
};

#endif //ARCHONCCD_H 
