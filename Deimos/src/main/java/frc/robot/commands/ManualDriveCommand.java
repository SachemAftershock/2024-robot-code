package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.PID;
import frc.lib.Util;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class ManualDriveCommand extends Command {

    final boolean showPrints = false;

    private final DriveSubsystem mDriveSubsystem;

    private final DoubleSupplier mTranslationXSupplier;
    private final DoubleSupplier mTranslationYSupplier;
    private final DoubleSupplier mRotationSupplier;

    public ManualDriveCommand(DriveSubsystem mDriveSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        this.mDriveSubsystem = mDriveSubsystem;
        this.mTranslationXSupplier = translationXSupplier;
        this.mTranslationYSupplier = translationYSupplier;
        this.mRotationSupplier = rotationSupplier;

        addRequirements(mDriveSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        mDriveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        mTranslationXSupplier.getAsDouble(),
                        mTranslationYSupplier.getAsDouble(),
                        rotationSpeedFromCardinalizerOrJoystickTwist(),
                        mDriveSubsystem.getGyroscopeRotation()));
    }

    @Override
    public void end(boolean interrupted) {
    }

    // Static variables for rotational cardinalization PID handling.
    // ManualDriveCommand doesn't end until interrupted, so the choice to
    // automatically cardinalize the direction needs to be statically altered. Note
    // that PID needs to be continuously updated.
    private static double mSetpointDegrees; // do not edit these directly. They are edited in setShouldCardinalize.
    private static PID mPid = new PID();
    private static boolean shouldCardinalize;
    // Design retrospective: It was perhaps possible to instead interrupt
    // ManualDriveCommand then re-call it with PID enabled towards a setpoint, but
    // this would likely make it harder to keep track of the variables for
    // DoubleSuppliers translation X and translation Y, since they
    // would need to be changed in both the default command ManualDriveCommand and
    // also the place in code where it would have been interrupted.
    // Bottom line: static interaction is needed to hook into rotationSupplier.

    /**
     * Override typical rotation supplier (right joystick twist) for a directional
     * cardinalization PID
     * 
     * @param shouldCardinalize true, if we want to override
     * @param setpointDegrees   this argument does not matter if you set
     *                          shouldCardinalize to false. Normalized within
     *                          method.
     */
    public static void setShouldCardinalize(boolean shouldCardinalize, double setpointDegrees) {
        ManualDriveCommand.shouldCardinalize = shouldCardinalize;
        mPid = new PID(); // reset PID
        mSetpointDegrees = Util.normalizeAngle(setpointDegrees);
        mPid.start(DriveConstants.kDriveAngularGains);
    }

    /**
     * If ManualDriveCommand is set to cardinalize, override joystick twist and use
     * cardinalization PID instead.
     * Forces back to normal joystick twist when setpoint is reached.
     * 
     * @return rotation speed
     */
    private double rotationSpeedFromCardinalizerOrJoystickTwist() {
        // use PID if cardinalization is desired
        double currentAngle = Util.normalizeAngle(mDriveSubsystem.getGyroscopeRotation().getDegrees());
        double rotationSpeed = mPid.updateRotation(currentAngle, mSetpointDegrees);

        mPid.updateRotation(currentAngle, mSetpointDegrees);
        if (Math.abs(mPid.getError()) < DriveConstants.kAutoRotateEpsilon && shouldCardinalize) {
            // PID finished, return to normal joystick twist
            System.out.println("Finished ManualDriveCommand cardinalization");
            ManualDriveCommand.setShouldCardinalize(false, 0);
        }
        if (shouldCardinalize) {
            return rotationSpeed;
        }
        // use typical joystick twist
        return mRotationSupplier.getAsDouble();
    }
}
