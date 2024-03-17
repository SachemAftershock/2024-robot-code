package frc.lib.recording;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.concurrent.ConcurrentLinkedQueue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * <p>
 * The class extending this should be a singleton.
 * <p>
 * It is recommended to read the HowToRecord.md docs before using.
 * 
 * <p>
 * <b>Example of extended class:</b>
 * 
 * <pre>
 * // Recorder.java
 * package frc.robot;
 * 
 * public class Recorder extends RecorderBase {
 *     private static Recorder mInstance; // singleton
 *     private DriveSubsystem mDriveSubsystem;
 * 
 *     private Recorder() {
 *         super(); // Don't leave this out
 *         mDriveSubsystem = DriveSubsystem.getInstance();
 *         // Initialize your subsystems in the
 *         // private constructor
 *     }
 * 
 *     // Define a logging function for use by others
 *     // outside the class, and ensure consistency and
 *     // readability. Pass its arguments to
 *     // internallyLogDoubles(double...)
 *     public void record(DriveSubsystem driveSubsystem) {
 *         ChassisSpeeds chassisSpeeds = driveSubsystem.getChassisSpeeds();
 * 
 *         // The .aftershockauto file is just a
 *         // list of doubles, so extract the doubles
 *         // as necessary before passing them into the
 *         // recorded values queue.
 *         internallyLogDoubles(
 *                 chassisSpeeds.vxMetersPerSecond,
 *                 chassisSpeeds.vyMetersPerSecond,
 *                 chassisSpeeds.omegaRadiansPerSecond);
 *     }
 * 
 *     // Since our .aftershockauto file recorded our
 *     // motor values every 20 milliseconds, this
 *     // will play them back every 20 milliseconds when
 *     // constantly called.
 *     // It is recommended to use the command wrapper
 *     // for this method, rather than publicizing it:
 *     // mRecorder.getRecordedAutonomousCommand()
 *     protected void playNextFrame() {
 *         // Get next recorded data entries then
 *         // remove head of queue
 *         double[] actions = getNextDoubles();
 * 
 *         // Set voltages and stuff with the doubles
 *         ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
 *                 actions[0], actions[1], actions[2]);
 *         mDriveSubsystem.drive(chassisSpeeds);
 *     }
 * 
 *     // singleton
 *     public synchronized static Recorder getInstance() {
 *         if (mInstance == null) {
 *             mInstance = new Recorder();
 *         }
 *         return mInstance;
 *     }
 * }
 * </pre>
 * 
 * @author Adrian Dayao
 */
public abstract class RecorderBase {

    // Queues are useful to put objects at the beginning and end of.
    private final ConcurrentLinkedQueue<double[]> mAutonomousLoggingQueue;
    private final ConcurrentLinkedQueue<double[]> mAutonomousPlaybackQueue;
    /**
     * Whether the recorder is playing. Use {@link RecorderBase#getIsPlaying()
     * getIsPlaying} to access this. Useful for stopping {@code periodic()}
     * functions in subsystems if they interfere with recorder playback.
     */
    private static boolean mIsPlaying;
    private Command mRecordingCreationCommand;

    /**
     * This constructor <b>must</b> be inherited via super, or else saving
     * recording data into our queues won't work. Intellisense doesn't catch that.
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
        mAutonomousLoggingQueue = new ConcurrentLinkedQueue<double[]>();
        mAutonomousPlaybackQueue = new ConcurrentLinkedQueue<double[]>();
        mIsPlaying = false;
        mRecordingCreationCommand = new InstantCommand(
                () -> DriverStation.reportWarning(
                        "Recorder: recordingCreationCommand was not initialized via startRecording",
                        false));
    }

    /**
     * Command to begin recording. Erases previous unsaved recordings.
     * 
     * <pre>
     * // Start a clean recording
     * Trigger startRecording = new Trigger(
     *         () -> mControllerPrimary.getRawButton(10));
     * startRecording.onTrue(
     *         mRecorder.startRecording(
     *                 () -> mRecorder.record(mDriveSubsystem)));
     * </pre>
     * 
     * Note that you need to implement your own {@code mRecorder.record(...)}.
     * 
     * @param recordFunctionLambda A runnable that you pass your Recorder.java's
     *                             {@code record()} method to.
     * @return a command to begin recording
     * @see #endRecording
     * @see #saveRecording
     */
    public final Command startRecording(Runnable recordFunctionLambda) {
        mRecordingCreationCommand.end(true);

        mRecordingCreationCommand = new FunctionalCommand(
                () -> { // init()
                    System.out.println("Recorder: Started recording.");
                    mAutonomousLoggingQueue.clear();
                },
                recordFunctionLambda, // execute()
                (Boolean interrupted) -> { // end()
                    System.out.println("Recorder: Stopped recording. Interrupted = " + interrupted);
                },
                () -> false, // isFinished()
                new Subsystem[0]); // subsystem requirements (none)

        return mRecordingCreationCommand;
    }

    /**
     * Command to end recording, whether one has started or not.
     * 
     * <pre>
     * // End a recording, but don't save it yet
     * Trigger endRecording = new Trigger(
     *         () -> mControllerPrimary.getRawButton(11));
     * endRecording.onTrue(mRecorder.endRecording());
     * </pre>
     * 
     * @return a command to stop recording
     * @see #startRecording
     * @see #saveRecording
     */
    public final Command endRecording() {
        return new InstantCommand(() -> {
            if (mRecordingCreationCommand.isFinished()) {
                DriverStation.reportError(
                        "Recorder: Tried to stop recording, but recording was already stopped.",
                        false);
            }
            mRecordingCreationCommand.cancel();
        });
    }

    /**
     * Wraps {@link RecorderBase#saveToFile(String) saveToFile} in an {@link
     * edu.wpi.first.wpilibj2.command.InstantCommand#InstantCommand(Runnable, Subsystem...)
     * InstantCommand}.
     * 
     * <pre>
     * // Save recording to RoboRIO /home/lvuser/MostRecent.aftershockauto
     * endRecording.onTrue(mRecorder.endRecording());
     * Trigger saveRecording = new Trigger(
     *         () -> mControllerPrimary.getRawButton(12));
     * saveRecording.onTrue(mRecorder.saveRecording("MostRecent"));
     * // BEWARE: If a file by this name preexists, it WILL be overwritten!
     * </pre>
     * 
     * @param fileNameToSaveTo File name to save to. Do not include the file
     *                         extension.
     * @return instant command to save to file.
     * @see #startRecording
     * @see #endRecording
     */
    public final Command saveRecording(String fileNameToSaveTo) {
        return new InstantCommand(() -> saveToFile(fileNameToSaveTo));
    }

    /**
     * <p>
     * Check if the recorder is playing back.
     * <p>
     * If your robot does
     * <i>not</i> use {@link #getRecordedAutonomousCommand(String, boolean)
     * getRecordedAutonomousCommand} to play a recording, you'll need to set
     * {@link #mIsPlaying isPlaying} manually
     * in RobotContainer's {@link frc.robot.Robot#autonomousInit() autonomousInit}
     * and {@link frc.robot.Robot#autonomousExit() autonomousExit}.
     * </p>
     * 
     * @return Whether the boolean {@code isPlaying} was set to true or false
     */
    public static final boolean getIsPlaying() {
        return mIsPlaying;
    }

    /**
     * <p>
     * Check if the recorder is playing back. If your robot does
     * <i>not</i> use {@code getRecordedAutonomousCommand()} to
     * play a recording, you'll need to set {@code isPlaying} manually in
     * {@link frc.robot.Robot#autonomousInit() autonomousInit} and
     * {@link frc.robot.Robot#autonomousExit() autonomousExit}.
     * </p>
     * 
     * @param recorderIsPlaying
     * @see #getIsPlaying
     */
    public static final void setIsPlaying(boolean recorderIsPlaying) {
        mIsPlaying = recorderIsPlaying; // yes this is static, but it's a singleton anyway
    }

    /**
     * Call this method within Recorder.java. The public logger, which talks to
     * this,
     * should have relevant, readable variable names, such as
     * <p>
     * {@code record(double driveSpeedX, double driveSpeedY)} or
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
     * @param doublesToLog the .aftershockauto file only knows how to save doubles
     */
    protected final void internallyLogDoubles(double... doublesToLog) {
        if (doublesToLog != null)
            mAutonomousLoggingQueue.add(doublesToLog);
    }

    /**
     * <p>
     * An autonomous file is saved like:
     * <p>
     * {@code 0.969161,0.531531,0.24219,9.593151} × 50 Hz × recording length.
     * <p>
     * Since the code only knows that the next entry in the queue of loaded
     * autonomous actions is a list of doubles, you'll have to implement what to do
     * with said doubles yourself in the class that extends {@code RecorderBase}.
     * 
     * @return Retrieves the head of the actions queue if it exists, Otherwise,
     *         return a 20-entry array of zeroes{@code [0.0, 0.0, 0.0...]}.
     *         Defaulting to these zeroes should only happen <i>after</i>
     *         the queue has finished reading.
     */
    protected final double[] getNextDoubles() {
        double[] next = mAutonomousPlaybackQueue.poll();
        return next != null ? next : new double[20];
    }

    /**
     * This function, which is composed into a command via
     * {@link RecorderBase.getRecordedAutonomousCommand()}, should be
     * structured as so.
     * 
     * <pre>
     * protected void playNextFrame() {
     *     // Read from next set of doubles in queue
     *     double actions = getNextDoubles();
     *     // Use the doubles to set voltages
     *     driveSubsystem.setSpeed(actions[0], actions[1]);
     *     // etc...
     * }
     * </pre>
     * 
     * Make sure to actually load an auto profile first via {@code initialize()},
     * and that the ordering of the array returned by getNextDoubles() is consistent
     * with your public-facing {@code record(...)} method.
     */
    abstract protected void playNextFrame();

    /**
     * Wraps {@link RecorderBase#loadFromFile(String, boolean) loadFromFile},
     * {@link RecorderBase#setIsPlaying(boolean) setIsPlaying}, and
     * {@link RecorderBase#playNextFrame() playNextFrame} in a command that
     * repeats until interrupted. Return this in RobotContainer's
     * {@link frc.robot.RobotContainer#getAutonomousCommand() getAutonomousCommand}.
     * 
     * <pre>
     * public Command getAutonomousCommand() {
     *     return mRecorder.getRecordedAutonomousCommand("MyAwesomeFile", false);
     * }
     * </pre>
     * 
     * @param fileNameToLoadFrom  the name of the .aftershockauto file to load from
     *                            the RoboRIO. Do not include a file extension.
     * @param fromDeployDirectory
     *                            <ul>
     *                            <li>True: The file is stored in the RoboRIO's
     *                            {@code /home/lvuser} directory, where recordings
     *                            by default get saved.
     *                            </li>
     *                            <li>False: The file is saved in the RoboRIO's
     *                            {@code /home/lvuser/deploy} directory. Files in
     *                            <i>our</i> {@code src/main/deploy} directory are
     *                            deployed onto the RoboRIO in said directory.
     *                            </li>
     *                            </ul>
     * @param requirements        the subsystems required by this command
     * @return constant (50Hz) repetition of {@code playNextFrame}, wrapped as a
     *         command.
     */
    public Command getRecordedAutonomousCommand(
            String fileNameToLoadFrom,
            boolean fromDeployDirectory,
            Subsystem... requirements) {

        // As of 2024-03-17, FunctionalCommand errors if null is passed as
        // Subsystem... varargs
        if (requirements == null)
            requirements = new Subsystem[0];
        System.out.println("Recorder: Trying to start autonomous");
        return new FunctionalCommand(
                () -> { // initialize()
                    // new Thread(() -> loadFromFile(fileNameToLoadFrom,
                    // fromDeployDirectory)).start();
                    loadFromFile(fileNameToLoadFrom, fromDeployDirectory);
                    System.out.println("Recorder: Started playing autonomous sequence " + fileNameToLoadFrom);
                    setIsPlaying(true);
                },
                () -> playNextFrame(), // execute()
                (Boolean interrupted) -> { // end(interrupted)
                    System.out.println(
                            "Recorder: Stopped playback of autonomous sequence " + fileNameToLoadFrom);
                    setIsPlaying(false);
                },
                () -> false, // isFinished()
                requirements); // no subsystems required. I might add a varargs argmuent "Subsystem...
                               // subsystems" to the getRecordedAutonomousCommand method instead of
                               // leaving this spot an empty array.
    }

    /**
     * Wraps {@link RecorderBase#loadFromFile(String, boolean) loadFromFile},
     * {@link RecorderBase#setIsPlaying(boolean) setIsPlaying}, and
     * {@link RecorderBase#playNextFrame() playNextFrame} in a command that
     * repeats until interrupted. Return this in RobotContainer's
     * {@link frc.robot.RobotContainer#getAutonomousCommand() getAutonomousCommand}.
     * 
     * <pre>
     * public Command getAutonomousCommand() {
     *     return mRecorder.getRecordedAutonomousCommand("MyAwesomeFile", false);
     * }
     * </pre>
     * 
     * @param fileNameToLoadFrom  the name of the .aftershockauto file to load from
     *                            the RoboRIO. Do not include a file extension.
     * @param fromDeployDirectory
     *                            <ul>
     *                            <li>True: The file is stored in the RoboRIO's
     *                            {@code /home/lvuser} directory, where recordings
     *                            by default get saved.
     *                            </li>
     *                            <li>False: The file is saved in the RoboRIO's
     *                            {@code /home/lvuser/deploy} directory. Files in
     *                            <i>our</i> {@code src/main/deploy} directory are
     *                            deployed onto the RoboRIO in said directory.
     *                            </li>
     *                            </ul>
     * @return constant (50Hz) repetition of {@code playNextFrame}, wrapped as a
     *         command.
     */
    public Command getRecordedAutonomousCommand(String fileNameToLoadFrom, boolean fromDeployDirectory) {
        return getRecordedAutonomousCommand(fileNameToLoadFrom, fromDeployDirectory, new Subsystem[0]);
    }

    /**
     * Save the current data to a .aftershockauto text file. If desired, you may
     * want to use PowerShell to <a href=
     * "https://docs.wpilib.org/en/stable/docs/software/roborio-info/roborio-ssh.html">
     * SSH</a> into the RoboRIO, to either keep a
     * <a href="https://cheat.sh/scp">SCP backup</a> of the file or to rename it
     * (with {@code mv}) after it is saved.
     * <p>
     * {@code > ssh lvuser@10.TE.AM.2} Start a remote SSH session on the RoboRIO
     * <p>
     * {@code > mv MostRecent.aftershockauto AwesomeFile.aftershockauto} While SSHed
     * into the RoboRIO, rename a file. Tab completion is recommended.
     * <p>
     * {@code > scp lvuser@10.TE.AM.2:AwesomeFile.aftershockauto .} Backup a file
     * from the RoboRIO to the current computer's working directory.
     * 
     * @param fileNameToSaveTo The name of the file to save to. Beware: if a file by
     *                         this name exists already, it <i>will</i> be deleted!
     * @see #initialize(String, String, boolean) initialize
     * @see #loadFromFile
     */
    public final void saveToFile(String fileNameToSaveTo) {
        System.out.println(
                "Recorder: Trying to save "
                        + fileNameToSaveTo
                        + ".aftershockauto to /home/lvuser");
        try {
            File autonomousDataFile = new File(
                    Filesystem.getOperatingDirectory(),
                    fileNameToSaveTo + ".aftershockauto");
            if (autonomousDataFile.exists()) {
                DriverStation.reportWarning(
                        "Recorder: Deleted previous "
                                + fileNameToSaveTo
                                + ".aftershockauto",
                        false);
                autonomousDataFile.delete();
            }
            autonomousDataFile.createNewFile();
            BufferedWriter fileWriter = new BufferedWriter(new FileWriter(autonomousDataFile));
            double[] autonomousDataEntry;
            while (true) {
                autonomousDataEntry = mAutonomousLoggingQueue.poll();
                if (autonomousDataEntry == null)
                    break;

                // .aftershockauto looks like
                // 0.52195219,0.52585219,0.53935219
                // 0.95225219,0.94295219,0.59995219
                for (int i = 0; i < autonomousDataEntry.length; i++) {
                    fileWriter.write(String.valueOf(autonomousDataEntry[i]) + ",");
                    if (i == autonomousDataEntry.length - 1) {
                        fileWriter.write(String.valueOf(autonomousDataEntry[i]) + "\n");
                    }
                    fileWriter.flush(); // make sure everything is written
                }
            }
            fileWriter.flush();
            fileWriter.close();
            System.out.println("Recorder: Saved auto queue data");
        } catch (IOException e) {
            e.printStackTrace();
        }
        mAutonomousLoggingQueue.clear();
    }

    /**
     * Set the current autonomous playback queue to the doubles stored in either
     * {@code /home/lvuser/fileNameToLoadFrom} or
     * {@code /home/lvuser/deploy/fileNameToLoadFrom}. It is <b>highly
     * recommended</b> to use the
     * {@link RecorderBase#getRecordedAutonomousCommand(String) command wrapper} for
     * this, instead of calling this method in {@code autonomousInit}.
     * 
     * 
     * @param fileNameToLoadFrom  the name of the .aftershockauto file to load from
     *                            the RoboRIO. Do not include a file extension.
     * @param fromDeployDirectory
     *                            <ul>
     *                            <li>True: The file is stored in the RoboRIO's
     *                            {@code /home/lvuser} directory, where recordings
     *                            get saved by default.
     *                            </li>
     *                            <li>False: The file is saved in the RoboRIO's
     *                            {@code /home/lvuser/deploy} directory, where files
     *                            in <i>our</i> {@code src/main/deploy} directory
     *                            get sent to.
     *                            </li>
     *                            </ul>
     */
    public final void loadFromFile(String fileNameToLoadFrom, boolean fromDeployDirectory) {

        // Get file as /home/lvuser/deploy/aftershockauto/%s.aftershockauto or
        // /home/lvuser/%s.aftershockauto
        File fileToRead;
        if (fromDeployDirectory) {
            fileToRead = new File(Filesystem.getDeployDirectory(), "aftershockauto");
            fileToRead = new File(fileToRead, fileNameToLoadFrom + ".aftershockauto");
        } else {
            fileToRead = new File(Filesystem.getOperatingDirectory(), fileNameToLoadFrom + ".aftershockauto");
        }

        System.out.println("Recorder: Trying to load file " + fileToRead.toString());
        mAutonomousPlaybackQueue.clear(); // Don't load multiple files into a playback queue
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
                mAutonomousPlaybackQueue.add(doubles);
                line = reader.readLine();
            }

            reader.close();

            System.out.println("Recorder: Loaded auto sequence " + fileToRead.toString() + " successfully");
        } catch (IOException e) {
            DriverStation.reportError("Recorder: Failed to find " + fileToRead.toString(), false);
        }
    }
}