# ssh quick reference

After saving a `.aftershockauto` file onto your bot, you may want to
manually rename it, or even back it up for later.

```bash
# Begin a remote SSH shell into the robot
C:\Users\You> ssh lvuser@10.TE.AM.2

# List files (we are in a minimal BusyBox/Linux now)
lvuser@roboRIO-263-FRC:~$ ls
MyRobot.jar*                                     aftershockauto/
MostRecent.aftershockauto                        deploy/
RightSide2noteWORKING.aftershockauto             frc/
MyKindOfMidPathRightSide2note.aftershockauto     frc_install/
LeftSideUnconfidentBounce.aftershockauto         logs/
Limelight.jar*                                   natinst/
networktables.json
robotCommand*

# Inspect a file's contents (use q to exit)
lvuser@roboRIO-263-FRC:~$ less MostRecent.aftershockauto

# copy a file to somewhere else on the bot
lvuser@roboRIO-263-FRC:~$ cp MostRecent.aftershockauto AmpSide2noteWORKING.aftershockauto

# change directories
lvuser@roboRIO-263-FRC:~$ cd aftershockauto
lvuser@roboRIO-263-FRC:~$ cd ..

# Logout (or use <control+D>)
lvuser@roboRIO-263-FRC:~$ exit 
```

To copy from the RoboRIO to your machine:

```powershell
# copy all .aftershockauto files to current dir via SCP
C:\Users\You> scp "lvuser@10.TE.AM.2:*.aftershockauto" "."
```