@echo off
setlocal EnableExtensions

set "NO_PAUSE="
if /I "%~1"=="--no-pause" set "NO_PAUSE=1"

echo.
echo ============================================
echo   IAR Build + Download: CM7_0 + CM7_1
echo   C-SPY is used with --download_only.
echo ============================================
echo.

call "%~dp0build_all_cores.bat" --no-pause
if errorlevel 1 (
    echo [FAIL] Build failed. Download is skipped.
    goto fail
)

call "%~dp0iar_env.bat"
if errorlevel 1 goto fail

call :download_core cyt4bb7_cm_7_0 CM7_0
if errorlevel 1 goto fail

call :download_core cyt4bb7_cm_7_1 CM7_1
if errorlevel 1 goto fail

echo.
echo ============================================
echo   Download complete. Debug session not opened.
echo ============================================
if not defined NO_PAUSE pause
exit /b 0

:download_core
set "GENERAL_XCL=%IAR_SETTINGS_DIR%\%~1.Debug.general.xcl"
set "DRIVER_XCL=%IAR_SETTINGS_DIR%\%~1.Debug.driver.xcl"
if not exist "%GENERAL_XCL%" (
    echo [FAIL] Missing C-SPY general config: "%GENERAL_XCL%"
    exit /b 1
)
if not exist "%DRIVER_XCL%" (
    echo [FAIL] Missing C-SPY driver config: "%DRIVER_XCL%"
    exit /b 1
)
echo [DOWNLOAD] %~2...
"%CSPYBAT_EXE%" -f "%GENERAL_XCL%" --download_only --backend -f "%DRIVER_XCL%"
if errorlevel 1 (
    echo [FAIL] %~2 download failed.
    exit /b 1
)
echo [OK] %~2 downloaded
echo.
exit /b 0

:fail
echo.
echo [FAIL] Download stopped.
if not defined NO_PAUSE pause
exit /b 1
