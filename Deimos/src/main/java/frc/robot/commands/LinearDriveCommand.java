package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


public class LinearDriveCommand extends Command {

    private DriveSubsystem mDrive;

    private double mDeltaY;
    private double mCurrentPoseY; 
    private TrapezoidProfile.Constraints m_constraintsY;
    private ProfiledPIDController m_controllerY;

    private double mDeltaX;
    private double mCurrentPoseX; 
    private TrapezoidProfile.Constraints m_constraintsX;
    private ProfiledPIDController m_controllerX;

    private double mDeltaZ;
    private double mCurrentPoseZ; 
    private TrapezoidProfile.Constraints m_constraintsZ;
    private ProfiledPIDController m_controllerZ;

    private int mIterationCounter;

    public LinearDriveCommand(DriveSubsystem drive, double deltaY, double deltaX) {
        this(drive,deltaY,deltaX,0);
    }

    public LinearDriveCommand(DriveSubsystem drive, double deltaY) {
        this(drive,deltaY,0);
    }

    // Field oriented command, X is cross field, Y is Downfield, Z is clockwise Azimuth positive
    public LinearDriveCommand(DriveSubsystem drive, double deltaY, double deltaX, double deltaAzimuth) {
        mDrive = drive;
        mDeltaY = deltaY;
        mDeltaX = deltaX;
        mDeltaZ = deltaAzimuth;

        m_constraintsY = new TrapezoidProfile.Constraints(DriveConstants.kMaxVelocityMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared);
        m_controllerY = new ProfiledPIDController(DriveConstants.kDriveLinearGains[0], DriveConstants.kDriveLinearGains[1], DriveConstants.kDriveLinearGains[2], m_constraintsY, DriveConstants.kDt);

        m_constraintsX = new TrapezoidProfile.Constraints(DriveConstants.kMaxVelocityMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared);
        m_controllerX = new ProfiledPIDController(DriveConstants.kDriveLinearGains[0], DriveConstants.kDriveLinearGains[1], DriveConstants.kDriveLinearGains[2], m_constraintsX, DriveConstants.kDt);

        m_constraintsZ = new TrapezoidProfile.Constraints(DriveConstants.kMaxAngularVelocityRadiansPerSecond, DriveConstants.kMaxAngularAccelerationRadiansPerSecondSquared);
        m_controllerZ = new ProfiledPIDController(DriveConstants.kDriveAngularGains[0], DriveConstants.kDriveAngularGains[1], DriveConstants.kDriveAngularGains[2], m_constraintsZ, DriveConstants.kDt);

        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        
        mCurrentPoseY = 0.0;
        mCurrentPoseY = mDrive.getPose().getTranslation().getY();
        mDeltaY += mDrive.getPose().getTranslation().getY();
        m_controllerY.reset(mCurrentPoseY);
        m_controllerY.setGoal(mDeltaY);

        mCurrentPoseX = 0.0;
        mCurrentPoseX = mDrive.getPose().getTranslation().getX();
        mDeltaX += mDrive.getPose().getTranslation().getX();
        m_controllerX.reset(mCurrentPoseX);
        m_controllerX.setGoal(mDeltaX);

        mCurrentPoseZ = 0.0;
        mCurrentPoseZ = mDrive.getPose().getRotation().getRadians();
        mDeltaZ += mDrive.getPose().getRotation().getRadians();
        m_controllerZ.reset(mCurrentPoseZ);
        m_controllerZ.setGoal(mDeltaZ);

        System.out.println("Linear Drive Command started : Current (" + mCurrentPoseY + ", " + mCurrentPoseX + ", " + mCurrentPoseZ + ") to Setpoint ( " + mDeltaY + ", " + mDeltaX + ", " + mDeltaZ*180.0/Math.PI +")");

        mIterationCounter = 0;
    }

    @Override
    public void execute() {
      
        mCurrentPoseY = mDrive.getPose().getTranslation().getY();
        double speedY = m_controllerY.calculate(mCurrentPoseY);

        mCurrentPoseX = mDrive.getPose().getTranslation().getX();
        double speedX = m_controllerX.calculate(mCurrentPoseX);

        mCurrentPoseZ = mDrive.getPose().getRotation().getRadians();
        double speedZ = m_controllerZ.calculate(mCurrentPoseZ);

        mIterationCounter++;

        if(mIterationCounter > 15) {
            mIterationCounter = 0;
            System.out.println("speed: (" + speedY + ", " + speedX + ", " + speedZ + "), Current (" + mCurrentPoseY + ", " + mCurrentPoseX + ", " + mCurrentPoseZ*180.0/Math.PI + ")");
        }

        mDrive.drive(new ChassisSpeeds(speedY, speedX, speedZ));
    }

    @Override
    public boolean isFinished() {
        mCurrentPoseY = mDrive.getPose().getTranslation().getY();
        mCurrentPoseX = mDrive.getPose().getTranslation().getX();
        mCurrentPoseZ = mDrive.getPose().getRotation().getRadians()*180.0/Math.PI;

        boolean acheivedY = Math.abs(mDeltaY - mCurrentPoseY) < DriveConstants.kLinearDriveTranslationEpsilon;
        boolean acheivedX = Math.abs(mDeltaX - mCurrentPoseX) < DriveConstants.kLinearDriveTranslationEpsilon;
        boolean acheivedZ = Math.abs(mDeltaZ - mCurrentPoseZ) < DriveConstants.kLinearDriveRotationEpsilon;

        return acheivedY && acheivedX && acheivedZ;
    }
    @Override
    public void end(boolean interrupted) {
        System.out.println("Linear Drive Command finished");
        mDrive.drive(new ChassisSpeeds());
    }
}