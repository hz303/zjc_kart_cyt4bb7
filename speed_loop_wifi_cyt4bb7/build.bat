@echo off
REM Command-line build for the DUAL-motor closed-loop WiFi project (no IDE needed).
REM Paths inside are ASCII-only on purpose: cmd.exe decodes .bat as GBK anyway.
set IARB=D:\IAR\common\bin\iarbuild.exe
set PRJ=%~dp0speed_loop_wifi\iar\project_config

if not exist "%IARB%" (
    echo [ERR] iarbuild.exe not found: %IARB%
    echo       Open speed_loop_wifi\iar\cyt4bb7.eww in IAR instead.
    pause
    exit /b 1
)

echo ============ CM7_0 ============
"%IARB%" "%PRJ%\cyt4bb7_cm_7_0.ewp" -build Debug
echo ============ CM7_1 ============
"%IARB%" "%PRJ%\cyt4bb7_cm_7_1.ewp" -build Debug

echo.
echo Output:
echo   %PRJ%\Debug_m7_0\Exe\cyt4bb7_cm_7_0.out  (.hex)
echo   %PRJ%\Debug_m7_1\Exe\cyt4bb7_cm_7_1.out  (.hex)
pause
