package frc.lib.recording;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Implement this as a singleton btw
 */
public abstract class RecorderAbstract {
    /**
     * No use yet
     */
    public static enum AutonomousBeginningPosition {
        redLeft, redCenter, redRight, blueLeft, blueCenter, blueRight
    }

    private final ConcurrentLinkedQueue<double[]> autonomousLoggingQueue;
    private final ConcurrentLinkedQueue<double[]> autonomousPlaybackQueue;
    private static boolean isPlaying;

    protected RecorderAbstract() {
        autonomousLoggingQueue = new ConcurrentLinkedQueue<double[]>();
        autonomousPlaybackQueue = new ConcurrentLinkedQueue<double[]>();
        isPlaying = false;
    }

    /**
     * Check if the recorder is playing back. Note that this must be set <b>manually</b>.
     * TODO finish docs
     * @return if the lines left to play back is 0. Note that recorder still "plays back",
     * but it just sets everything to 0.
     */
    public static final boolean getIsPlaying() {
        return isPlaying;
    }

    /**
     * Set this in Robot.java's autonomousInit and Robot.java's autonomousExit
     * @param recorderIsPlaying
     */
    public static final void setIsPlaying(boolean recorderIsPlaying) {
        isPlaying = recorderIsPlaying;
    }

    /**
     * Only call this internally. An external logger should have relevant,
     * readable variable names, such as
     * <p>
     * {@code record(double speedX, double speedY)} or
     * <p>
     * {@code record(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem)}
     * 
     * @param doublesToLog
     */
    protected final void internallyLogDoubles(double... doublesToLog) {
        if (doublesToLog != null)
            autonomousLoggingQueue.add(doublesToLog);
    }

    /**
     * 
     * @return Retrieves the head of the queue, or the array [0.0, 0.0, 0.0...]
     */
    protected final double[] getNextDoubles() {
        double[] next = autonomousPlaybackQueue.poll();
        return next != null ? next : new double[20];
    }

    /**
     * This function, which should be called in autonomous periodic, should be
     * structured as so:
     * 
     * <pre>
     *public void playNextFrame() {
     *  double actions = getNextDoubles(); // Read from next set of doubles in queue
     *  driveSubsystem.setSpeed(actions[0], actions[1]);
     *  // etc...
     *}
     * </pre>
     * 
     * Make sure to actually load an auto profile first (via {@code loadFile}), and that
     * the ordering of your arguments is consistent.
     */
    abstract public void playNextFrame();

    /**
     * Saves the current data collected by the auton recorder to the specified file
     * path and clears the AutonRecorder.
     * 
     * @param autonomousDataFileName The path to the file to save the auton data to.
     */
    public void saveToFile(String autonomousDataFileName) {
        try {
            File autonomousDataFile = new File(String.format("/home/lvuser/%s.txt", autonomousDataFileName));
            if (autonomousDataFile.exists()) {
                autonomousDataFile.delete();
            }
            autonomousDataFile.createNewFile();
            BufferedWriter mWriter = new BufferedWriter(new FileWriter(autonomousDataFile));
            // CHANGE THE NAME OF THE TEXT FILE BASED OFF OF WHAT AUTON YOU ARE WRITING FOR

            // Write all Doubles[]
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
     * Set the desired action list to the given file name as stored in RoboRIO.
     * 
     * @param autonomousDataFileName
     */
    public final void loadFromFile(String autonomousDataFileName) {
        System.out.printf("Recorder: Trying to load file %s.txt\n", autonomousDataFileName);
        autonomousPlaybackQueue.clear();
        try {
            BufferedReader reader = new BufferedReader(
                    new FileReader(String.format("/home/lvuser/%s.txt", autonomousDataFileName)));

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

            System.out.println("Recorder: Load auto's queue successfully");
        } catch (IOException e) {
            System.out.printf("Recorder: Failed to find %s.txt\n", autonomousDataFileName);
        }
    }

    // /**
    // * <p>
    // * Begin a recording at the current time of all ChassisSpeeds values.
    // * </p>
    // * <p>
    // * Stored in a CSV file in the RoboRIO. Use
    // * <a href=
    // *
    // "https://docs.wpilib.org/en/stable/docs/software/roborio-info/roborio-ssh.html">
    // * SSH </a>
    // * to retrieve.
    // * </p>
    // *
    // * @param recordOrNot true to begin recording, false to stop. Remember to call
    // * Recorder.periodic()
    // * @throws IOException
    // */
    // public void setIsRecording(boolean recordOrNot) {
    // // Stop recording if false is passed
    // if (!recordOrNot) {
    // try {
    // if (mWriter != null) {
    // mWriter.flush();
    // mWriter.close();
    // }
    // } catch (IOException e) {
    // System.err.println("Close failure: "+e);
    // }
    // return;
    // }

    // // Skip init on true if initialized already
    // if (mWriterInitialized)
    // return;

    // try {
    // mWriter = new FileWriter(String.format("/home/lvuser/recordedAuto_%s.csv",
    // new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss").format(new Date())));
    // } catch (Exception e) {
    // System.err.println("FAILED to initialize recorder: "+e);
    // return;
    // }
    // mWriterStartTime = System.currentTimeMillis();
    // mWriterInitialized = true;
    // }

    // private void addToCSV(String... args) throws IOException {
    // String s = "";
    // for (int i = 0; i < args.length - 1; i++)
    // s = s + args[i] + ",";
    // s += "\n";
    // mWriter.append(s);
    // }

    // /**
    // * Updates
    // */
    // public void periodic() throws IOException {
    // if (mWriter != null && mWriterInitialized) {

    // rec(
    // // Time, ChassisSpeedsX, ChassisSpeedsY, ChassisSpeedsOmega,
    // Objects.toString(System.currentTimeMillis() - mWriterStartTime, null),
    // );
    // // start each "frame" with the elapsed time since we started recording
    // mWriter.append("" + (System.currentTimeMillis() - mWriterStartTime));
    // }
    // }
}