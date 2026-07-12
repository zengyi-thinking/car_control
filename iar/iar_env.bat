@echo off
rem Shared IAR command-line environment for this workspace.
rem Optional override: set IAR_EW_ROOT=D:\path\to\Embedded Workbench 9.2

set "IAR_SCRIPT_DIR=%~dp0"
for %%I in ("%IAR_SCRIPT_DIR%.") do set "IAR_SCRIPT_DIR=%%~fI\"
set "IAR_PROJECT_DIR=%IAR_SCRIPT_DIR%project_config"
set "IAR_SETTINGS_DIR=%IAR_PROJECT_DIR%\settings"

if not exist "%IAR_PROJECT_DIR%" (
    echo [FAIL] IAR project_config folder not found: "%IAR_PROJECT_DIR%"
    exit /b 1
)

set "IARBUILD_EXE="
set "CSPYBAT_EXE="

if defined IAR_EW_ROOT (
    if exist "%IAR_EW_ROOT%\common\bin\iarbuild.exe" (
        set "IARBUILD_EXE=%IAR_EW_ROOT%\common\bin\iarbuild.exe"
    )
    if exist "%IAR_EW_ROOT%\common\bin\CSpyBat.exe" (
        set "CSPYBAT_EXE=%IAR_EW_ROOT%\common\bin\CSpyBat.exe"
    )
)

if not defined IARBUILD_EXE (
    for %%T in (iarbuild.exe) do set "IARBUILD_EXE=%%~$PATH:T"
)
if not defined CSPYBAT_EXE (
    for %%T in (CSpyBat.exe cspybat.exe) do if not defined CSPYBAT_EXE set "CSPYBAT_EXE=%%~$PATH:T"
)

if not defined IARBUILD_EXE (
    if exist "D:\software\IAR Systems\Embedded Workbench 9.2\common\bin\iarbuild.exe" (
        set "IARBUILD_EXE=D:\software\IAR Systems\Embedded Workbench 9.2\common\bin\iarbuild.exe"
    )
)
if not defined CSPYBAT_EXE (
    if exist "D:\software\IAR Systems\Embedded Workbench 9.2\common\bin\CSpyBat.exe" (
        set "CSPYBAT_EXE=D:\software\IAR Systems\Embedded Workbench 9.2\common\bin\CSpyBat.exe"
    )
)

if not defined IARBUILD_EXE (
    echo [FAIL] iarbuild.exe not found. Set IAR_EW_ROOT or add IAR common\bin to PATH.
    exit /b 1
)
if not defined CSPYBAT_EXE (
    echo [FAIL] CSpyBat.exe not found. Set IAR_EW_ROOT or add IAR common\bin to PATH.
    exit /b 1
)

exit /b 0
