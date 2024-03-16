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
 * The class extending this should be a singleton, and interacted with
 * during {@code autonomousInit}, {@code autonomousPeriodic}, and
 * {@code autonomousExit}.
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
 *     // for this method, rather than publicizing it.
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
    private final ConcurrentLinkedQueue<double[]> autonomousLoggingQueue;
    private final ConcurrentLinkedQueue<double[]> autonomousPlaybackQueue;
    private static boolean isPlaying;
    /**
     * Whether {@link RecorderBase#initialize(String, String, boolean) initialize}
     * was called
     */
    private boolean mInitialized;
    private String mAutonomousLoggingFileName; // should be initialized via
    private String mAutonomousPlaybackFileName; // initialize()
    private boolean mLoadPlaybackFromDeployDirectory;

    private FunctionalCommand mRecordingCreationCommand;

    /**
     * This constructor <b>must</b> be inherited via super, or else saving
     * the data into our queues won't work. Intellisense doesn't catch that.
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
        mAutonomousLoggingFileName = "PLACEHOLDER_NAME";
        mAutonomousPlaybackFileName = "PLACEHOLDER_NAME";

        mRecordingCreationCommand = new InstantCommand(
                () -> DriverStation.reportWarning("Recorder: recordingCreationCommand was not initialized", false));
    }

    /**
     * Call this in Robot.java's {@code robotInit} to select which files to save and
     * load your recorded auto sequence from
     * 
     * @param fileToSaveRecordingTo           the name of the
     *                                        {@code .aftershockauto} file to save a
     *                                        recording to
     * @param fileToLoadRecordingFrom         the name of the
     *                                        {@code .aftershockauto} file to load a
     *                                        recording from
     * @param loadPlaybackFromDeployDirectory Whether the recording to play back was
     *                                        deployed from the computer into the
     *                                        RoboRIO's {@code /home/lvuser/deploy}
     *                                        directory (true), or the recording is
     *                                        stored in the RoboRIO's default
     *                                        {@code /home/lvuser} home directory
     *                                        (false). Newly-saved auto sequence
     *                                        files are initially stored in the
     *                                        {@code /home/lvuser} directory.
     */
    public final void initialize(
            String fileToSaveRecordingTo,
            String fileToLoadRecordingFrom,
            boolean loadPlaybackFromDeployDirectory) {
        System.out.println("Recorder: Initialized.");
        if (mInitialized) {
            DriverStation.reportWarning("Recorder: Initialized a second time.", false);
        }
        mInitialized = true;
        mAutonomousLoggingFileName = fileToSaveRecordingTo;
        mAutonomousPlaybackFileName = fileToLoadRecordingFrom;
        mLoadPlaybackFromDeployDirectory = loadPlaybackFromDeployDirectory;
        loadFromFile();
        System.out.println("Recorder: Savefile name = " + mAutonomousLoggingFileName);
        System.out.println("Recorder: Playback file name = " + mAutonomousPlaybackFileName);
        System.out.println("Recorder: Playback file is expected to be in /home/lvuser"
                + (mLoadPlaybackFromDeployDirectory ? "/deploy" : ""));
    }

    /**
     * Command to begin recording. Erases previous unsaved recordings.
     * 
     * <pre>
     * // Start a clean recording
     * Trigger beginRecording = new Trigger(
     *         () -> mControllerPrimary.getRawButton(10));
     * beginRecording.onTrue(
     *         mRecorder.beginRecording(
     *                 () -> mRecorder.record(mDriveSubsystem)));
     * </pre>
     * 
     * Note that you need to implement your own {@code mRecorder.record(...)}.
     * 
     * @param recordFunctionLambda A runnable that you pass Record.java's
     *                             {@code record()}
     *                             method to.
     * @return a command to begin recording
     * @see endRecording
     * @see saveRecording
     */
    public final Command beginRecording(Runnable recordFunctionLambda) {
        mRecordingCreationCommand.end(true);

        mRecordingCreationCommand = new FunctionalCommand(
                () -> {
                    System.out.println("Recorder: Began recording.");
                    autonomousLoggingQueue.clear();
                }, // init()
                recordFunctionLambda,
                (Boolean interrupted) -> {
                    System.out.println("Recorder: Stopped recording. Interrupted = " + interrupted);
                }, // end()
                () -> false, // isFinished()
                new Subsystem[0]); // subsystem requirements (none)

        return mRecordingCreationCommand;
    }

    /**
     * Command to end recording, whether it has started or not.
     * 
     * <pre>
     * // End a recording, but don't save it yet
     * Trigger endRecording = new Trigger(
     *         () -> mControllerPrimary.getRawButton(11));
     * endRecording.onTrue(mRecorder.endRecording());
     * </pre>
     * 
     * @return a command to stop recording
     * @see #beginRecording
     * @see #saveRecording
     */
    public final Command endRecording() {
        return new InstantCommand(() -> {
            if (mRecordingCreationCommand.isFinished()) {
                DriverStation.reportError(
                        "Recorder: Tried to stop current recording, but it was already stopped.",
                        false);
            } else {
                mRecordingCreationCommand.end(true);
            }
        });
    }

    /**
     * Wraps {@link RecorderBase#saveToFile() saveToFile()} in an InstantCommand.
     * The file name is determined by
     * {@link RecorderBase#initialize(String, String, boolean) initialize}.
     * 
     * <pre>
     * // Save the recording to the file declared in
     * // mRecorder.initialize() in robotInit
     * endRecording.onTrue(mRecorder.endRecording());
     * Trigger saveRecording = new Trigger(
     *         () -> mControllerPrimary.getRawButton(12));
     * saveRecording.onTrue(mRecorder.saveRecording());
     * </pre>
     * 
     * @return instant command to save to file.
     * @see #beginRecording
     * @see #saveRecording
     */
    public final Command saveRecording() {
        return new InstantCommand(() -> saveToFile());
    }

    /**
     * <p>
     * Check if the recorder is playing back. If your robot does
     * <i>not</i> use {@code getRecordedAutonomousCommand()} to
     * play a recording, you'll need to set this manually in
     * {@link frc.robot.Robot#autonomousInit() autonomousInit} and
     * {@link frc.robot.Robot#autonomousExit() autonomousExit}.
     * </p>
     * 
     * @return Whether the boolean {@code isPlaying} was set to true or false
     */
    public static final boolean getIsPlaying() {
        return isPlaying;
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
        isPlaying = recorderIsPlaying; // yes this is static, but it's a singleton anyway
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
            autonomousLoggingQueue.add(doublesToLog);
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
        double[] next = autonomousPlaybackQueue.poll();
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
     * Wraps {@link RecorderBase#loadFromFile()} and
     * {@link RecorderBase#playNextFrame() playNextFrame()} in a repeated,
     * cancellable command. Return this in RobotContainer's
     * {@code getAutonomousCommand()}
     * 
     * <pre>
     * public Command getAutonomousCommand() {
     *     return mRecorder.getRecordedAutonomousCommand();
     * }
     * </pre>
     * 
     * @return constant (50Hz) repetition of {@code playNextFrame}, wrapped as a
     *         command. Also keeps track of isPlaying
     */
     
    public Command getRecordedAutonomousCommand() {
        System.out.println("Recorder: Trying to start autonomous");
        return new FunctionalCommand(
                () -> { // init()
                    System.out.println("Recorder: Started playing autonomous sequence " + mAutonomousPlaybackFileName);
                    setIsPlaying(true);
                },
                () -> playNextFrame(), // execute()
                (Boolean interrupted) -> { // end(interrupted)
                    System.out.println(
                            "Recorder: Stopped playback of autonomous sequence " + mAutonomousPlaybackFileName);
                    setIsPlaying(false);
                },
                () -> false, // isFinished()
                new Subsystem[0]); // no subsystems required. This may change in the future, to make required
                                   // systems a varargs.
    }

    /**
     * Save the current data to a .aftershockauto text file, as defined by
     * {@link RecorderBase#initialize(String, String, boolean) initialize}, within
     * the RoboRIO. If desired, you may want to <a href=
     * "https://docs.wpilib.org/en/stable/docs/software/roborio-info/roborio-ssh.html">
     * SSH</a> into the RoboRIO to keep a <a href="https://cheat.sh/scp">backup</a>
     * of the file via PowerShell.
     * <p>
     * {@code > ssh lvuser@10.TE.AM.2}
     * 
     * @see #initialize(String, String, boolean) initialize
     * @see #loadFromFile
     */
    public final void saveToFile() {
        System.out.println(
                "Recorder: Trying to save "
                        + mAutonomousLoggingFileName
                        + ".aftershockauto to /home/lvuser");
        try {
            File autonomousDataFile = new File(
                    Filesystem.getOperatingDirectory(),
                    mAutonomousLoggingFileName + ".aftershockauto");
            if (autonomousDataFile.exists()) {
                DriverStation.reportWarning(
                        "Recorder: Deleted previous "
                                + mAutonomousLoggingFileName
                                + ".aftershockauto",
                        false);
                autonomousDataFile.delete();
            }
            autonomousDataFile.createNewFile();
            BufferedWriter fileWriter = new BufferedWriter(new FileWriter(autonomousDataFile));
            double[] autonomousDataEntry;
            while (true) {
                autonomousDataEntry = autonomousLoggingQueue.poll();
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
        autonomousLoggingQueue.clear();
    }

    /**
     * Set the current autonomous playback queue to whatever file you have saved on
     * the RoboRIO that goes by the name defined in
     * {@link #initialize(String, String, boolean) initialize}.
     * 
     * @see #saveToFile
     * @see #initialize
     */
    protected final void loadFromFile() {

        // Get file as /home/lvuser/deploy/%s.aftershockauto or
        // /home/lvuser/%s.aftershockauto
        File fileToRead = new File(mLoadPlaybackFromDeployDirectory ? Filesystem.getDeployDirectory()
                : Filesystem.getOperatingDirectory(), mAutonomousPlaybackFileName + ".aftershockauto");

        System.out.println("Recorder: Trying to load file " + fileToRead);
        autonomousPlaybackQueue.clear(); // Don't load multiple files into a playback queue
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

            System.out.println("Recorder: Loaded auto sequence " + fileToRead.toString() + " successfully");
        } catch (IOException e) {
            DriverStation.reportError("Recorder: Failed to find " + fileToRead, false);
        }
    }
}