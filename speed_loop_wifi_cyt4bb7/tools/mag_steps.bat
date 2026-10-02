@echo off
chcp 936 >nul
cd /d "%~dp0"
"C://Users//zjc39//.workbuddy//binaries//python//envs//default//Scripts//python.exe" mag_steps.py %1
echo.
echo [done] data saved to tools\mag_steps.csv
pause
