@echo off
chcp 936 >nul
cd /d "%~dp0"
"C://Users//zjc39//.workbuddy//binaries//python//envs//default//Scripts//python.exe" mag_trace.py %1
echo.
echo [done] trace saved to tools\mag_trace.csv
pause
