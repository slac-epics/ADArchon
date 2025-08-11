ADArchon Releases
==================

The latest untagged master branch can be obtained at
/afs/slac/g/cd/swe/git/repos/package/epics/modules/ADArchon.git.

The versions of EPICS base, asyn, and other synApps modules used for each release can be obtained from 
the EXAMPLE_RELEASE_PATHS.local, EXAMPLE_RELEASE_LIBS.local, and EXAMPLE_RELEASE_PRODS.local
files respectively, in the configure/ directory of the appropriate release of the 
[top-level areaDetector](https://github.com/areaDetector/areaDetector) repository.


Release Notes
=============
R1.0.7 2025-08-11 Jane Liu
------------------
* Updated modules to latest releases

R1.0.0 (29-May-2024)
------------------
* Support for full frame and full vertical binning modes.
* Basic support for frame batching to enable fast running.
* Support for modifying settings for HeaterX and XVBias modules from the IOC.

R1.0.1 (15-Aug-2024)
------------------
* Add expert screen for XVBias module.
* Add support for using per line batch timestamps if controller supports it -> more reliable timestamsp in batch mode.

R1.0.2 (16-Oct-2024)
------------------
* Add short PV alias for HeaterX and XVBias module PVs.
* Update the version of ADCore and asyn.

Future Releases
===============
* Add support for missing module types.
* Add readback of the remaining configuration parameters from the ACF file.
