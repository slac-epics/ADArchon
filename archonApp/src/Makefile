TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

LIBRARY_IOC_WIN32 += archonCCD
LIBRARY_IOC_Linux += archonCCD
LIB_SRCS += archonCCD.cpp camera_ts.cpp
LIB_LIBS_WIN32 += archon

DBD += archonCCDSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

