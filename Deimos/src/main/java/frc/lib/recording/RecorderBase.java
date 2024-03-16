package frc.lib.recording;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.concurrent.ConcurrentLinkedQueue;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * <p>
 * The class extending this should be a singleton, and interacted with
 * during {@code autonomousInit}, {@code autonomousPeriodic}, and
 * {@code autonomousExit}.
 * </p>
 * 
 * <b>Example of extended class:</b>
 * 
<pre>
public class Recorder extends RecorderBase {
    private static Recorder mInstance; // singleton
    private DriveSubsystem mDriveSubsystem;

    private Recorder() {
        super(); // Don't leave this out
        mDriveSubsystem = DriveSubsystem.getInstance();
        // Initialize your subsystems in the private constructor
    }

    // Define a logging function for use outside of the class,
    // according to your code needs. Pass its
    // arguments to internallyLogDoubles(double...)
    public void record(DriveSubsystem driveSubsystem) {
        ChassisSpeeds chassisSpeeds = driveSubsystem.getChassisSpeeds();

        internallyLogDoubles(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond);
    }

    // For use in autonomousPeriodic, after an auto file was loaded
    public void playNextFrame() {
        double[] actions = getNextDoubles(); // recorded data entries
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(actions[0], actions[1], actions[2]);
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
</pre>

<b> Playing a recording in Robot.java </b>

<pre>

&#64;Override
public void autonomousInit() {
    mRecorder.loadFromFile("My file to save");
    Recorder.setIsPlaying(true);
}

@Override
public void autonomousPeriodic() {
    mRecorder.playNextFrame();
}

@Override
public void autonomousExit() {
    Recorder.setIsPlaying(false);
}
</pre>

<b> Making a recording. (Command-based example) </b>

<pre>
private Command loggingCommand = new InstantCommand(
        () -> mRecorder.record(mDriveSubsystem, mShooterSubsystem, mIntakeSubsystem)).repeatedly();

private void configureButtonBindings() {

    Trigger beginRecording = new Trigger(() -> mControllerPrimary.getRawButton(10));
    beginRecording.onTrue(new InstantCommand(() -> {
        System.out.println("Recorder: began recording");
    }).andThen(loggingCommand));
    Trigger endRecording = new Trigger(() -> mControllerPrimary.getRawButton(11));
    endRecording.onTrue(new InstantCommand(() -> {
        System.out.println("Recorder: ended recording");
        loggingCommand.cancel();
    }));
    Trigger saveRecording = new Trigger(() -> mControllerPrimary.getRawButton(12));
    saveRecording
            .onTrue(new InstantCommand(() -> mRecorder.saveToFile(Recorder.AutonomousBeginningPosition.redCenter)));
}
</pre>
 * 
 * @author Adrian Dayao
 */
public abstract class RecorderBase {
    /**
     * <p>
     * Used with {@link RecorderBase#loadFromFile(AutonomousBeginningPosition)} and
     * {@link RecorderBase#saveToFile(AutonomousBeginningPosition)}
     */
    public static enum AutonomousBeginningPosition {
        redLeft, redCenter, redRight, blueLeft, blueCenter, blueRight
    }

    private final ConcurrentLinkedQueue<double[]> autonomousLoggingQueue;
    private final ConcurrentLinkedQueue<double[]> autonomousPlaybackQueue;
    private static boolean isPlaying;

    // /**
    //  * This method shouldn't be here tbh. Breaks encapsulation. FIXME remove this and any need to reference it
    //  */
    // public void clearAutonomousLoggingQueue() {
    //     autonomousLoggingQueue.clear();
    // }

    /**
     * This constructor must be inherited via super, or else the queues won't
     * work. Intellisense doesn't catch that.
     * 
     * <pre>
     *public class Recorder extends RecorderBase() {
     *    private RecorderBase() {
     *        super();
     *    } 
     *}
     * </pre>
     */
    protected RecorderBase() {
        autonomousLoggingQueue = new ConcurrentLinkedQueue<double[]>();
        autonomousPlaybackQueue = new ConcurrentLinkedQueue<double[]>();
        isPlaying = false;
    }

    /**
     * <p>
     * Check if the recorder is playing back. Note that the boolean claiming whether
     * the recorder is playing back must be set <b>manually</b>
     * via setIsPlaying(), which is typically done in the {@code autonomousInit}
     * and {@code autonomousExit} override methods in Robot.java.
     * </p>
     * 
     * @return Whether the boolean {@code isPlaying} was set to true or false
     */
    public static final boolean getIsPlaying() {
        return isPlaying;
    }

    /**
     * Set this in Robot.java's autonomousInit and Robot.java's autonomousExit
     * 
     * @param recorderIsPlaying Static, but a Recorder should be a singleton anyway.
     */
    public static final void setIsPlaying(boolean recorderIsPlaying) {
        isPlaying = recorderIsPlaying;
    }

    /**
     * Call this method within Recorder. The public logger, which talks to this,
     * should have relevant, readable variable names, such as
     * <p>
     * {@code record(double speedX, double speedY)} or
     * <p>
     * {@code record(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem)}
     * <p>
     * <b>Example:</b>
     * 
     * <pre>
     * // within class Recorder.java extends RecorderBase
     *public void record(
     *    DriveSubsystem driveSubsystem,
     *    IntakeSubsystem intakeSubsystem
     *) {
     *    ChassisSpeeds chassisSpeeds = driveSubsystem.getChassisSpeeds();
     * 
     *    internallyLogDoubles(
     *        chassisSpeeds.vxMetersPerSecond,
     *        chassisSpeeds.vyMetersPerSecond,
     *        chassisSpeeds.omegaRadiansPerSecond,
     *        mIntakeSubsystem.getIntakeArmMotorSpeed()
     * }
     * </pre>
     * 
     * @param doublesToLog
     */
    protected final void internallyLogDoubles(double... doublesToLog) {
        if (doublesToLog != null)
            autonomousLoggingQueue.add(doublesToLog);
    }

    /**
     * <p>
     * An autonomous file is saved like:
     * {@code 0.969161,0.531531,0.24219,9.593151} × 50 Hz × recording length.
     * <p>
     * Since the code only knows that the next entry in the queue of loaded
     * autonomous actions is a list of doubles, you'll have to implement what to do
     * with them yourself in the class that extends {@code RecorderBase}.
     * 
     * @return Retrieves the head of the actions queue if it exists, Otherwise,
     *         return a 20-entry array of zeroes{@code [0.0, 0.0, 0.0...]}
     */
    protected final double[] getNextDoubles() {
        double[] next = autonomousPlaybackQueue.poll();
        return next != null ? next : new double[20];
    }

    /**
     * This function, which should be called in {@code autonomousPeriodic}, should
     * be structured as so:
     * 
     * <pre>
     * public void playNextFrame() {
     *     double actions = getNextDoubles(); // Read from next set of doubles in queue
     *     driveSubsystem.setSpeed(actions[0], actions[1]);
     *     // etc...
     * }
     * </pre>
     * 
     * Make sure to actually load an auto profile first (via {@code loadfromFile}),
     * and
     * that the ordering of your arguments is consistent.
     */
    abstract public void playNextFrame();

    /**
     * Save the current data to a text file by this name, within the RoboRIO. If
     * desired, you may want to <a href=
     * "https://docs.wpilib.org/en/stable/docs/software/roborio-info/roborio-ssh.html">
     * SSH</a> into the RoboRIO to keep a <a href="https://cheat.sh/scp">backup</a>
     * of the file via PowerShell.
     * <p>
     * {@code > ssh lvuser@10.TE.AM.2}
     * 
     * @param autonomousDataFileName The path to the file to save the auton data to.
     * @see #loadFromFile
     */
    public final void saveToFile(String autonomousDataFileName) {
        try {
            File autonomousDataFile = new File(Filesystem.getOperatingDirectory(),
                    autonomousDataFileName + ".aftershockauto");
            if (autonomousDataFile.exists()) {
                autonomousDataFile.delete();
            }
            autonomousDataFile.createNewFile();
            BufferedWriter mWriter = new BufferedWriter(new FileWriter(autonomousDataFile));
            double[] autonomousDataEntry;
            while (true) {
                autonomousDataEntry = autonomousLoggingQueue.poll();
                if (autonomousDataEntry == null)
                    break;

                for (int i = 0; i < autonomousDataEntry.length; i++) {
                    mWriter.write(String.valueOf(autonomousDataEntry[i]) + ",");
                    if (i == autonomousDataEntry.length - 1) {
                        mWriter.write(String.valueOf(autonomousDataEntry[i]) + "\n");
                    }
                    mWriter.flush();
                }
            }
            mWriter.flush();
            mWriter.close();
            System.out.println("Recorder: Save auto queue data");
        } catch (IOException e) {
            e.printStackTrace();
        }
        autonomousLoggingQueue.clear();
    }

    /**
     * Save the current data to a text file by this <i>enum's</i> name,
     * within the RoboRIO. If desired, you may want to <a href=
     * "https://docs.wpilib.org/en/stable/docs/software/roborio-info/roborio-ssh.html">
     * SSH</a> into the RoboRIO to keep a <a href="https://cheat.sh/scp">backup</a>
     * of the filevia PowerShell.
     * <p>
     * {@code > ssh lvuser@10.TE.AM.2}
     * 
     * @param autonomousDataFileName The path to the file to save the auton data to.
     */
    public final void saveToFile(AutonomousBeginningPosition autonomousDataFileNameByPosition) {
        saveToFile(autonomousDataFileNameByPosition.toString());
    }

    /**
     * Set the current autonomous playback queue to whatever file you have saved on
     * the RoboRIO that goes by this name. This should be called in
     * {@code autonomousInit}.
     * 
     * @param autonomousDataFileName
     * @param fromDeployDirectory    Defaults to false. If true, load the file we
     *                               sent to the RoboRIO during the deploy step,
     *                               rather than the previous recording.
     *                               This file would be stored in
     *                               {@code src/main/deploy/AftershockAuto}
     * @see #saveToFile
     */
    public final void loadFromFile(String autonomousDataFileName, boolean fromDeployDirectory) {

        // Get file as /home/lvuser/deploy/%s.aftershockauto or
        // /home/lvuser/%s.aftershockauto
        File fileToRead = new File(fromDeployDirectory ? Filesystem.getDeployDirectory()
                : Filesystem.getOperatingDirectory(), autonomousDataFileName + ".aftershockauto");

        System.out.println("Recorder: Trying to load file " + fileToRead);
        autonomousPlaybackQueue.clear();
        try {
            BufferedReader reader = new BufferedReader(
                    new FileReader(fileToRead));
            System.out.println("Reading " + fileToRead.toString());

            // Turn the file into a FIFO queue of double arrays
            String line = reader.readLine();
            while (line != null) {
                // stringDoubles[i] like "421.2426,12601.9,612.6"
                String stringDoubles[] = line.split(",");
                double[] doubles = new double[stringDoubles.length];
                for (int i = 0; i < stringDoubles.length; i++)
                    doubles[i] = Double.parseDouble(stringDoubles[i]);
                autonomousPlaybackQueue.add(doubles);
                line = reader.readLine();
            }

            reader.close();

            System.out.println("Recorder: Loaded auto queue successfully");
        } catch (IOException e) {
            System.out.println("Recorder: Failed to find " + fileToRead);
        }
    }

    /**
     * Set the current autonomous playback queue to whatever file you have saved on
     * the RoboRIO that goes by this name. This should be called in
     * {@code autonomousInit}.
     * 
     * @param autonomousDataFileName
     * @see #saveToFile
     */
    public final void loadFromFile(String autonomousDataFileName) {
        loadFromFile(autonomousDataFileName, false);
    }

    /**
     * Set the current autonomous playback queue to whatever file you have saved on
     * the RoboRIO that goes by this <i>enum's</i> name. This should be called in
     * {@code autonomousInit}.
     * 
     * @param autonomousDataFileNameByPosition the enum in
     *                                         RecorderBase.{@link AutonomousBeginningPosition}
     * @see #saveToFile
     */
    public final void loadFromFile(AutonomousBeginningPosition autonomousDataFileNameByPosition) {
        loadFromFile(autonomousDataFileNameByPosition.toString(), false);
    }

    /**
     * Set the current autonomous playback queue to whatever file you have saved on
     * the RoboRIO that goes by this <i>enum's</i> name. This should be called in
     * {@code autonomousInit}.
     * 
     * @param autonomousDataFileNameByPosition the enum in
     *                                         RecorderBase.{@link AutonomousBeginningPosition}
     * @param fromDeployDirectory              Defaults to false. If true, load the
     *                                         file we
     *                                         sent to the RoboRIO during the deploy
     *                                         step,
     *                                         rather than the previous recording.
     *                                         This file would be stored in
     *                                         {@code src/main/deploy/AftershockAuto}
     * @see #saveToFile
     */
    public final void loadFromFile(AutonomousBeginningPosition autonomousDataFileNameByPosition,
            boolean fromDeployDirectory) {
        loadFromFile(autonomousDataFileNameByPosition.toString(), false);
    }
}