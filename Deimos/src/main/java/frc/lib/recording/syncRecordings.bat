@echo off
@REM Change to the directory of this batch file
cd /D %~dp0


set "source=/home/lvuser/*.aftershockauto"
set "destination=../../../../"

echo Copying all .aftershockauto files to this recording/ directory
scp lvuser@10.2.63.2:%source% %destination%


echo You may want to move them into the deploy/ directory
echo to shove them into the bot. WIP automate this 

@REM SSH into the bot with ssh lvuser@10.2.63.2