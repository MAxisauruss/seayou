@echo off
rem ---------------------------------------------------------------------
rem  SeaYou ML view - the drone camera with the detection model's boxes.
rem
rem  Double-click this on the ground-station laptop, AFTER the ground
rem  station is running. Leave the window open for the demo; close it to
rem  stop. Then open Live Feeds and click the ML VIEW tile.
rem
rem  Extra options go straight through, e.g. to demo with no drone:
rem      start_ml_view.bat --source 0              (a USB webcam)
rem      start_ml_view.bat --source clip.mp4       (a video, looped)
rem ---------------------------------------------------------------------
cd /d "%~dp0"
title SeaYou ML view

if not exist ".venv-ml\Scripts\python.exe" (
    echo.
    echo The ML environment is not installed yet. From this folder, run once:
    echo.
    echo     python -m venv .venv-ml
    echo     .venv-ml\Scripts\python.exe -m pip install -r requirements-ml.txt
    echo.
    echo It is about a 350 MB download. Delete the .venv-ml folder to remove it.
    echo.
    pause
    exit /b 1
)

".venv-ml\Scripts\python.exe" ml_view.py %*
echo.
echo The ML view has stopped.
pause
