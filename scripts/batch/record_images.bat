REM Script for recording images directly from Optris in windows

set output_dir=%USERPROFILE%\Documents\GitHub\thermalvis\output\images

%USERPROFILE%\Documents\GitHub\BUILDS\thermalvis-32\apps\win-optris\Release\OptrisWinTest.exe "%output_dir%"

REM pause
cmd /k
