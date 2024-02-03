package frc.robot.commands.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.DriveConstants.kOffBalanceAngleThresholdDegrees;
import static frc.robot.Constants.DriveConstants.kOnBalanceAngleThresholdDegrees;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;

public class ManualDriveCommand extends Command {
    private final DriveSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final AHRS ahrs = new AHRS(SPI.Port.kMXP, (byte) 200);

    public ManualDriveCommand(DriveSubsystem drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    boolean autoBalanceXMode;
    boolean autoBalanceYMode;
    double pitchAngleDegrees;
    double rollAngleDegrees;
    double xAxisRate;
    double yAxisRate;

    @Override
    public void execute() {
        antiTilt();

        while (!autoBalanceYMode && !autoBalanceXMode) {
            m_drivetrainSubsystem.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            m_translationXSupplier.getAsDouble(),
                            m_translationYSupplier.getAsDouble(),
                            m_rotationSupplier.getAsDouble(),
                            m_drivetrainSubsystem.getGyroscopeRotation()));
        }

    }

    private void antiTilt() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        pitchAngleDegrees = ahrs.getPitch();
        rollAngleDegrees = ahrs.getRoll();

        if (!autoBalanceXMode &&
                (Math.abs(pitchAngleDegrees) >= Math.abs(kOffBalanceAngleThresholdDegrees))) {
            autoBalanceXMode = true;
        } else if (autoBalanceXMode &&
                (Math.abs(pitchAngleDegrees) <= Math.abs(kOnBalanceAngleThresholdDegrees))) {
            autoBalanceXMode = false;
        }
        if (!autoBalanceYMode &&
                (Math.abs(pitchAngleDegrees) >= Math.abs(kOffBalanceAngleThresholdDegrees))) {
            autoBalanceYMode = true;
        } else if (autoBalanceYMode &&
                (Math.abs(pitchAngleDegrees) <= Math.abs(kOnBalanceAngleThresholdDegrees))) {
            autoBalanceYMode = false;
        }

        // Control drive system automatically,
        // driving in reverse direction of pitch/roll angle,
        // with a magnitude based upon the angle

        if (autoBalanceXMode) {
            double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
            xAxisRate = Math.sin(pitchAngleRadians) * -1;
        }
        if (autoBalanceYMode) {
            double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
            yAxisRate = Math.sin(rollAngleRadians) * -1;
        }
        m_drivetrainSubsystem.drive(new ChassisSpeeds(xAxisRate, yAxisRate, 0));

    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
