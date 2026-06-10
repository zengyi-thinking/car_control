@echo off
setlocal

set "IAR_ROOT=D:\software\IAR Systems\Embedded Workbench 9.2"
set "IAR_BIN=%IAR_ROOT%\common\bin"
set "PROJ_DIR=%~dp0project_config"

echo.
echo [BUILD] Compiling both cores...
echo.

echo [BUILD] Core0 (CM7_0)...
"%IAR_BIN%\iarbuild.exe" "%PROJ_DIR%\cyt4bb7_cm_7_0.ewp" -make Debug -log warnings
if %ERRORLEVEL% NEQ 0 (
    echo [FAIL] Core0 build failed!
    exit /b 1
)
echo [OK] Core0 build success
echo.

echo [BUILD] Core1 (CM7_1)...
"%IAR_BIN%\iarbuild.exe" "%PROJ_DIR%\cyt4bb7_cm_7_1.ewp" -make Debug -log warnings
if %ERRORLEVEL% NEQ 0 (
    echo [FAIL] Core1 build failed!
    exit /b 1
)
echo [OK] Core1 build success
echo.

echo ============================================
echo   Build complete!
echo ============================================
