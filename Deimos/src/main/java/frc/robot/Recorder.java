package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.recording.RecorderBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * <p>
 * Call getIsPlaying() to see if the recorder is playing.
 * </p>
 * <p>
 * Note that isPlaying is manually set via setIsPlaying(bool) in
 * AutonomousPeriodic
 * and AutonomousExit
 * </p>
 */
public class Recorder extends RecorderBase {
    private static Recorder mInstance;
    private DriveSubsystem mDriveSubsystem;
    private IntakeSubsystem mIntakeSubsystem;
    private ShooterSubsystem mShooterSubsystem;

    private Recorder() {
        super();
        mDriveSubsystem = DriveSubsystem.getInstance();
        mIntakeSubsystem = IntakeSubsystem.getInstance();
        mShooterSubsystem = ShooterSubsystem.getInstance();
    }

    public void record(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem,
            IntakeSubsystem intakeSubsystem) {
        ChassisSpeeds chassisSpeeds = driveSubsystem.getChassisSpeeds();

        internallyLogDoubles(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond,
                mIntakeSubsystem.getIntakeArmMotorSpeed(),
                mIntakeSubsystem.getRollerMotorSpeed(),
                mShooterSubsystem.getShooterMotorSpeed()[0],
                mShooterSubsystem.getShooterMotorSpeed()[1]);
    }

    public void playNextFrame() {
        double[] actions = getNextDoubles(); // recorded data entries
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(actions[0], actions[1], actions[2]);
        mDriveSubsystem.drive(chassisSpeeds);
        mIntakeSubsystem.setIntakeArmMotorSpeed(actions[3]);
        mIntakeSubsystem.setRollerMotorSpeed(actions[4]);
        mShooterSubsystem.setShooterMotorSpeed(actions[5], actions[6]);
    }

    public synchronized static Recorder getInstance() {
        if (mInstance == null) {
            mInstance = new Recorder();
        }
        return mInstance;
    }
}
