package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.PID;
import frc.lib.Util;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class RotateDriveCommand extends Command {

    final boolean showPrints = false;		

    private DriveSubsystem mDrive;
    private double mSetpointDegrees;
    private PID mPid;
    
    /**
     * Field Relative Rotation, downfield is 0deg
     */
    public RotateDriveCommand(DriveSubsystem drive, double setpointDegrees) {
        mDrive = drive;
        mSetpointDegrees = setpointDegrees;
        mPid = new PID();
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        mPid.start(DriveConstants.kDriveAngularGains);
        if (showPrints) System.out.println("RotateDriveCommand started " + Double.toString(mSetpointDegrees) + " degrees.");
    }

    @Override
    public void execute() {
        double currentAngle = Util.normalizeAngle(mDrive.getGyroscopeRotation().getDegrees());
		double rotationSpeed = mPid.updateRotation(currentAngle, mSetpointDegrees)
				* DriveConstants.kMaxAngularVelocityRadiansPerSecond * 0.5;
        
        mDrive.drive(new ChassisSpeeds(0, 0, rotationSpeed));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(mPid.getError()) < DriveConstants.kAutoRotateEpsilon;
    }
    @Override
    public void end(boolean interrupted) {
        mDrive.drive(new ChassisSpeeds());
    }
}