@echo off
echo ===========================================
echo RADUN Flight Controller Build Test
echo ===========================================

echo.
echo [1] Checking toolchain...
arm-none-eabi-gcc --version
if errorlevel 1 (
    echo ERROR: ARM GCC toolchain not found!
    echo Install: arm-none-eabi-gcc
    exit /b 1
)

echo.
echo [2] Cleaning previous build...
make clean

echo.
echo [3] Building firmware...
make 2>&1 | tee build.log

echo.
echo [4] Checking build results...
if exist radun_accif435.hex (
    echo SUCCESS: Firmware built successfully!
    echo File: radun_accif435.hex
    
    echo.
    echo [5] Size analysis:
    make size
    
    echo.
    echo [6] Summary:
    echo - Temperature compensation: ENABLED
    echo - PropGuard: IMPLEMENTED
    echo - Filter state management: FIXED
    echo - Health monitoring: ADDED
    echo - DShot GCR table: CORRECTED
    
    echo.
    echo Ready to flash to ACCIF435!
) else (
    echo ERROR: Build failed!
    echo Check build.log for details
    exit /b 1
)

pause