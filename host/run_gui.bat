@echo off
rem mymotor FOC upper computer launcher
rem The FOC board talks on USART1 (PB6=TX / PB7=RX, 115200 8N1).
rem Use the standalone USB-TTL adapter (e.g. CP210x) -- NOT the DAPLink VCP,
rem which crashes under sustained telemetry traffic.
setlocal
set PY="C:\Users\Gary\AppData\Local\Programs\Python\Python312\python.exe"
cd /d "%~dp0"
if "%~1"=="" (
    %PY% foc_gui.py
) else (
    %PY% foc_gui.py %1
)
endlocal
