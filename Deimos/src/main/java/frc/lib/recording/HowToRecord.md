# RecorderBase.java

## Introduction

Hey, welcome to the Recorder.java how-to. In VSCode, press the following keys to view this document in the correct formatting.

Press `<CTRL+K>`, then `V`.

## Goals

`RecorderBase.java` is an abstract class meant to handle the
unchanging logic of the reading, writing, queueing, and loading,
so that your custom `Recorder.java` is only filled with the
changeable logic *you* care about.

**Internally**, our design is basically as such:

`Read voltages -> put doubles into queue -> write queue to text file`

`Read text file -> put doubles into queue -> write voltages`

You only need to worry about reading and writing the voltages.

<small>
This guide is designed for the CommandRobot paradigm. It
is possible to replicate it in a TimedRobot style, but
you will need to call many of the command-wrapped methods
individually.

<details>
<summary> How can I do this with a TimedRobot? </summary>

<b>Many methods were abstracted into commands</b>.  

You'll have to dig
and find each method to call individually, to replicate the
effect of said commands.

```java
// Robot.java
@Override
Recorder mRecorder = Recorder.getInstance();
public void robotInit() {
    mRecorder.initialize(
      "FILE_TO_SAVE_TO", 
      "FILE_TO_READ_FROM", 
      false
    );
}
@Override
public void autonomousInit() {
    mRecorder.loadFromFile(); // abstracted
    Recorder.setIsPlaying(true); // abstracted
}

@Override
public void autonomousPeriodic() {
    mRecorder.playNextFrame(); // abstracted, though it is
    // defined manually in Recorder.java
}

@Override
public void autonomousExit() {
    Recorder.setIsPlaying(false); // abstracted
}
```



</details>

</small>

## Quick start (copy-pastable code)

### Example Recorder.java implementation

This records a SwerveDrive's chassis speeds (mDriveSubsystem),
then plays them back.

```java
// Recorder.java
package frc.robot;

public class Recorder extends RecorderBase {
    private static Recorder mInstance; // singleton
    private DriveSubsystem mDriveSubsystem;

    private Recorder() {
        super(); // Don't leave this out
        mDriveSubsystem = DriveSubsystem.getInstance();
        // Initialize your subsystems in the
        // private constructor
    }

    // Define a logging function for use by others
    // outside the class, and ensure consistency and 
    // readability. Pass its arguments to
    //      internallyLogDoubles(double...)
    public void record(DriveSubsystem driveSubsystem) {
        ChassisSpeeds chassisSpeeds =
            driveSubsystem.getChassisSpeeds();

        // The .aftershockauto file is just a
        // list of doubles, so extract the doubles
        // as necessary before passing them into the
        // recorded values queue.
        internallyLogDoubles(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond);
    }

    // Since our .aftershockauto file recorded our
    // motor values every 20 milliseconds, this
    // will play them back every 20 milliseconds when
    // constantly called.
    // It is recommended to use the command wrapper
    // for this method, rather than publicizing it.
    //      mRecorder.getRecordedAutonomousCommand()
    protected void playNextFrame() {
        // Get next recorded data entries then
        // remove head of queue
        double[] actions = getNextDoubles(); 

        // Set voltages and stuff with the doubles
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            actions[0], actions[1], actions[2]);
        mDriveSubsystem.drive(chassisSpeeds);
    }

    // singleton
    public synchronized static Recorder getInstance() {
        if (mInstance == null) {
            mInstance = new Recorder();
        }
        return mInstance;
    }
}
```

### Initialize the file names to save and write to

```java
// Robot.java
private Recorder mRecorder = Recorder.getInstance();

@Override
public void robotInit() {
    mRecorder.initialize(
        "fileToSaveTo", // .aftershockauto file in bot
        "fileToLoadFrom",
        false // Whether the file you're loading was
              // deployed from your computer (true),
              // or was written within the bot (false).
    );
}

```

### Create an auto sequence file (fileToSaveTo.aftershockauto)

You'll have to pass your custom `mRecorder.record()` method
in via a runnable.

```java
// RobotContainer.java
private Recorder mRecorder = Recorder.getInstance();

  private void configureButtonBindings() {

    // Start a clean recording
    Trigger beginRecording = new Trigger(
        () -> mControllerPrimary.getRawButton(10)
    );
    beginRecording.onTrue(mRecorder.beginRecording(
        () -> mRecorder.record(mDriveSubsystem)
    ));
    // End a recording, but don't save it yet
    Trigger endRecording = new Trigger(
        () -> mControllerPrimary.getRawButton(11)
    );
    // Save the recording to the file declared in
    // mRecorder.initialize() in robotInit
    endRecording.onTrue(mRecorder.endRecording());
    Trigger saveRecording = new Trigger(
        () -> mControllerPrimary.getRawButton(12)
    );
    saveRecording.onTrue(mRecorder.saveRecording());

    // rest of your bindings, etc...
}
```

### Play an auto sequence file (fileToLoadFrom.aftershockauto)

```java
// RobotContainer.java
private Recorder mRecorder = Recorder.getInstance();

public Command getAutonomousCommand() {
    // Play the recording saved in the file declared in
    // mRecorder.initialize() in robotInit
    return mRecorder.getRecordedAutonomousCommand();
}
```

## SSH, and how files are stored on the RoboRIO

After saving a `.aftershockauto` file onto your bot, you may want to
manually rename it, or even back it up for later.

```powershell
# Begin a remote SSH shell into the robot
$ ssh lvuser@10.TE.AM.2

# List files (we are in a minimal BusyBox/Linux now)
$ ls

# Inspect a file's contents (use q to exit)
$ less TESTFILE.aftershockauto

# copy a file to somewher else on the bot
$ cp TESTFILE.aftershockauto WorkingCopyLeftSide.aftershockauto

# Logout (or use <control+D>)
$ exit 
```

To copy to your machine:

```powershell
# copy all .aftershockauto files to current dir via SCP
$ scp "lvuser@10.TE.AM.2:*.aftershockauto" "."

```