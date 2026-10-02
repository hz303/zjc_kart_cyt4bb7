@echo off
chcp 936 >nul
cd /d "%~dp0"
"C:\Users\zjc39\.workbuddy\binaries\python\envs\default\Scripts\python.exe" mag_sweep.py sweep 25 %1
echo.
echo [done] raw data saved next to this file (csv)
pause
