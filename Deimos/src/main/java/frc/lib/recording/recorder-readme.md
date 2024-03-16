# recordings list

## Switch the recording to play back

Go to Robot.java --> RobotInit --> method `loadFromFile(filename)`

### What's good for loadFromFile()?

| file name (do not include .aftershockauto extension) | description                              |
|------------------------------------------------------|------------------------------------------|
| BlueSourceRedAmpTwoVALIDATED                         | when you face the speaker, this is left  |
| SpeakerCenterSideTwoNoteVERIFIED                     | center duh                               |
| SpeakerRightSideTwoNoteVERIFIED                      | when you face the speaker, this is right |
| OneNote                                              | untested tbh try not to use this         |

Note that you'll have to re-deploy the robot if you change the filename.
Be sure to watch the print statements in console -- there are printlines
when a file *is* loaded and when it *cannot* be loaded.

## My recording was awesome, how do I save it with a useful name?

By default, files are saved as "MostRecent.aftershockauto" in the
RoboRIO's Linux home directory `/home/lvuser`.  

<small>
The default save name MostRecent is defined via `saveToFile`,
called in RobotContainer.
</small>

Because saving will overwrite any preexisting files of the same name,
it's recommended to keep MostRecent as the default, but rename any
awesome recordings manually.

```bash
# Start a remote SSH shell into the RoboRIO's operating system
C:\Users\you> ssh lvuser@10.2.63.2

# List all files to see output in /home/lvuser, aka the ~ directory
lvuser@roboRIO-263-FRC:~$ ls
202403151844.aftershockauto                      SpeakerCenterSideTwoNoteVERIFIED.aftershockauto
BlueAmpRedSourceTwoVALIDATED.aftershockauto      SpeakerLeftSideTwoNoteFailed.aftershockauto
BlueSourceRedAmpTwoVALIDATED.aftershockauto      SpeakerRightSideTwoNoteVERIFIED.aftershockauto
Deimos.jar*                                      TESTFILE.aftershockauto
FRC_UserProgram.log@                             _tmp035a5cbf89a2423dbbafecab91dd6a1a.et.md5
LeftSideAgain.aftershockauto                     deploy/
LeftSideAgainPlayback.aftershockauto             frc/
LeftSideTest.aftershockauto                      frc_install/
LeftSideUnconfidentBounce.aftershockauto         logs/
Limelight.jar*                                   natinst/
OneNote.aftershockauto                           networktables.json
README_File_Paths.txt@                           robotCommand*
ReallyGoodLeftSide.aftershockauto

# Rename a file by "moving" it
lvuser@roboRIO-263-FRC:~$ mv MostRecent.aftershockauto BlueAmpRedSourceFiveVALIDATED.aftershockauto

# Leave SSH.
lvuser@roboRIO-263-FRC:~$ exit
```

### I wanna stop using recorder playback.

There isn't an easy way to do this. I tried to refactor the code to make this ezpz but... :cri_everytiem:

```java
// Robot.java

public void autonomousInit() {
    // Comment this out
    mRecorder.loadFromFile("BlueAmpRedSourceTwoVALIDATED", false);

    // Uncomment this in
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Comment this out
    Recorder.setIsPlaying(true);
}

@Override
public void autonomousPeriodic() {
    // Comment this out
    mRecorder.playNextFrame();
}

public void autonomousExit() {
    // Keep this
    Recorder.setIsPlaying(false);
}
```

To go back to using Recorder again, simply perform the comment steps in reverse.