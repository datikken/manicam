@echo off
REM Build and run the host-side unit tests.
REM
REM Requires a native C++17 compiler on PATH. Uses clang++ when present
REM (it is what this project was developed against) and falls back to g++.
REM
REM No PlatformIO involvement: the tests link against the test/support shims
REM for Arduino.h / Servo.h / Wire.h / EEPROM.h, so the real AVR libraries
REM never enter the picture.

setlocal enabledelayedexpansion

set SRC=src\Axis\Axis.cpp src\AxisDriver\AxisDriver.cpp src\Joystick\Joystick.cpp src\Link\JoystickFrame.cpp src\Link\LinkMaster.cpp src\Link\LinkSlave.cpp src\PositionStore\PositionStore.cpp

set TESTS=test\test_main.cpp test\test_axis.cpp test\test_axis_driver.cpp test\test_joystick_frame.cpp test\test_joystick.cpp test\test_position_store.cpp test\test_link_slave.cpp test\test_link_master.cpp

where clang++ >nul 2>nul
if %ERRORLEVEL%==0 (
    set CXX=clang++
    set LINKOPTS=-Xlinker /subsystem:console
) else (
    where g++ >nul 2>nul
    if %ERRORLEVEL%==0 (
        set CXX=g++
        set LINKOPTS=
    ) else (
        echo No C++ compiler found ^(need clang++ or g++^).
        exit /b 1
    )
)

echo Using !CXX!
!CXX! -std=c++17 -Wall -Wextra -I test\support -I src -o test\run_tests.exe !TESTS! !SRC! !LINKOPTS!
if errorlevel 1 (
    echo.
    echo BUILD FAILED
    exit /b 1
)

echo.
test\run_tests.exe
set RESULT=%ERRORLEVEL%
exit /b %RESULT%
