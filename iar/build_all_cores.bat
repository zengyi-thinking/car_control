@echo off
setlocal EnableExtensions

set "NO_PAUSE="
if /I "%~1"=="--no-pause" set "NO_PAUSE=1"

call "%~dp0iar_env.bat"
if errorlevel 1 goto fail

echo.
echo ============================================
echo   IAR Build: CM7_0 + CM7_1
echo ============================================
echo.

call :build_core cyt4bb7_cm_7_0 CM7_0
if errorlevel 1 goto fail

call :build_core cyt4bb7_cm_7_1 CM7_1
if errorlevel 1 goto fail

echo.
echo ============================================
echo   Build complete.
echo ============================================
if not defined NO_PAUSE pause
exit /b 0

:build_core
echo [BUILD] %~2...
"%IARBUILD_EXE%" "%IAR_PROJECT_DIR%\%~1.ewp" -make Debug -log warnings
if errorlevel 1 (
    echo [FAIL] %~2 build failed.
    exit /b 1
)
echo [OK] %~2 build success
echo.
exit /b 0

:fail
echo.
echo [FAIL] Build stopped.
if not defined NO_PAUSE pause
exit /b 1
