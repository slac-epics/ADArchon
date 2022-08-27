TOP=..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# Note, the files in this directory were manually copied from LCLS1 DAQ system implementation
# of the archon controller readout library. It's essentially a libary made out of the Drive.cpp
# file from the DAQ.

# The extension on archon.hh was changed from .hh to .h because the EPICS build system does
# not recognize .hh
INC += archon.h

ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
LIB_INSTALLS_WIN32    += ../os/win32-x86/archon.lib

else ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
LIB_INSTALLS_WIN32    += ../os/windows-x64/archon.lib

else ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
LIB_INSTALLS_Linux += ../os/linux-x86_64/libarchon.so

else ifeq (rhel6-x86_64, $(findstring rhel6-x86_64, $(T_A)))
LIB_INSTALLS_Linux += ../os/linux-x86_64/libarchon.so

else ifeq (rhel7-x86_64, $(findstring rhel7-x86_64, $(T_A)))
LIB_INSTALLS_Linux += ../os/rhel7-x86_64/libarchon.so

else ifeq (linux-x86, $(findstring linux-x86, $(T_A)))
LIB_INSTALLS_Linux += ../os/linux-x86/libarchon.so

endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

