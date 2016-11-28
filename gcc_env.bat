@ECHO off
ECHO Adding cmake and mingw32 path to PATH
SET CMAKEBIN=C:\nxp\cmake\bin
SET MINGW32BIN=C:\MinGW\bin
ECHO %PATH%|findstr /i /c:"%CMAKEBIN:"=%">nul || set PATH=%PATH%;%CMAKEBIN%
ECHO %PATH%|findstr /i /c:"%MINGW32BIN:"=%">nul || set PATH=%PATH%;%MINGW32BIN%
ECHO %PATH%

@rem set where to find the ARM launchpad compiler and tools
@rem SET ARMGCC_DIR=C:\temp\toradex\4.9_2015q3
SET ARMGCC_DIR=C:\nxp\KDS_3.2.0\Toolchain

@rem build examples with build_all.bat inside the demos