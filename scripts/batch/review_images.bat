REM Script for reviewing images recorded from Optris in windows (using the "record_images.bat" script)

set launch_file=%USERPROFILE%\Documents\GitHub\thermalvis\launch\optris_review.launch

%USERPROFILE%\Documents\GitHub\BUILDS\thermalvis\apps\mono-slam\Release\MonocularSLAM.exe "%launch_file%"

REM pause
cmd /k
