/**
 * Area Detector driver for the Archon CCD.
 *
 * @author Daniel Damiani
 * @date Aug 2022
 */

#ifndef ARCHONCCD_H
#define ARCHONCCD_H

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

    // Should be private, but are called from C so must be public
    //void statusTask(void);
    //void dataTask(void);

  protected:

  private:

    bool checkStatus(bool status, const char *message);

    epicsEventId statusEvent;
    epicsEventId dataEvent;
    Pds::Archon::Driver *mDrv;
    bool mExiting;
    int mExited;
    bool mInitOK;
};

#endif //ARCHONCCD_H 
