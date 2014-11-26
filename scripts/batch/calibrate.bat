REM Script for reviewing images recorded from Optris in windows (using the "record_images.bat" script)

set launch_file=%USERPROFILE%\Documents\GitHub\thermalvis\launch\calibrate_optris.launch

%USERPROFILE%\Documents\GitHub\BUILDS\thermalvis\apps\mm-calibrator\Release\MM-Calibrator.exe "%launch_file%"

REM pause
cmd /k
