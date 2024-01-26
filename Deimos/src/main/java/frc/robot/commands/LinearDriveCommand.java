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

    public LinearDriveCommand(DriveSubsystem drive, double deltaX, double deltaY) {
        this(drive,deltaX,deltaY,0);
    }

    public LinearDriveCommand(DriveSubsystem drive, double deltaX) {
        this(drive,deltaX,0);
    }

    // Field oriented command, X is cross field, Y is Downfield, Z is clockwise Azimuth positive
    //deltaAzimuth is degrees
    public LinearDriveCommand(DriveSubsystem drive, double deltaX, double deltaY, double deltaAzimuth) {
        mDrive = drive;
        mDeltaX = deltaX;
        mDeltaY = deltaY;
        mDeltaZ = deltaAzimuth;

        System.out.println("Delta X " + deltaX + " Delta Y " + deltaY + " Delta Radians " + mDeltaZ);

        m_constraintsX = new TrapezoidProfile.Constraints(DriveConstants.kMaxVelocityMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared);
        m_controllerX = new ProfiledPIDController(DriveConstants.kDriveLinearGains[0], DriveConstants.kDriveLinearGains[1], DriveConstants.kDriveLinearGains[2], m_constraintsX, DriveConstants.kDt);

        m_constraintsY = new TrapezoidProfile.Constraints(DriveConstants.kMaxVelocityMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared);
        m_controllerY = new ProfiledPIDController(DriveConstants.kDriveLinearGains[0], DriveConstants.kDriveLinearGains[1], DriveConstants.kDriveLinearGains[2], m_constraintsY, DriveConstants.kDt);

        m_constraintsZ = new TrapezoidProfile.Constraints(100.0 * Math.PI, 100.0 * Math.PI);
        //new TrapezoidProfile.Constraints(DriveConstants.kMaxAngularVelocityRadiansPerSecond, DriveConstants.kMaxAngularAccelerationRadiansPerSecondSquared);
        m_controllerZ = new ProfiledPIDController(DriveConstants.kDriveAngularGains[0], DriveConstants.kDriveAngularGains[1], DriveConstants.kDriveAngularGains[2], m_constraintsZ, DriveConstants.kDt);

        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        
        mCurrentPoseX = 0.0;
        mCurrentPoseX = mDrive.getPose().getX();  //.getTranslation().getX();
        mDeltaX += mCurrentPoseX;
        m_controllerX.reset(mCurrentPoseX);
        m_controllerX.setGoal(mDeltaX);

        mCurrentPoseY = 0.0;
        mCurrentPoseY = mDrive.getPose().getY();  //.getTranslation().getY();
        mDeltaY += mCurrentPoseY;
        m_controllerY.reset(mCurrentPoseY);
        m_controllerY.setGoal(mDeltaY);
        
        mCurrentPoseZ = 0.0;
        mCurrentPoseZ = mDrive.getGyroscopeRotation().getDegrees();
        mDeltaZ += mCurrentPoseZ;
        m_controllerZ.reset(mCurrentPoseZ);
        m_controllerZ.setGoal(mDeltaZ);

        System.out.println("Linear Drive Command started : Current (" + mCurrentPoseX + ", " + mCurrentPoseY + ", " + mCurrentPoseZ + ") to Setpoint ( " + mDeltaX + ", " + mDeltaY + ", " + mDeltaZ + ")");

        mIterationCounter = 0;
    }

    @Override
    public void execute() {
 
        mCurrentPoseX = mDrive.getPose().getX();
        double speedX = m_controllerX.calculate(mCurrentPoseX);

        mCurrentPoseY = mDrive.getPose().getY();
        double speedY = m_controllerY.calculate(mCurrentPoseY);

        mCurrentPoseZ =  mDrive.getGyroscopeRotation().getDegrees();; //  mCurrentPoseZ = mDrive.getGyroscopeRotation().getRadians();
        double speedZ = m_controllerZ.calculate(mCurrentPoseZ);
        //double speedZ = 0.0;

        mIterationCounter++;

        // if(mIterationCounter > 15) {
        //     mIterationCounter = 0;
        //     System.out.printf("speed: (%.6f, %.6f, %.6f), current (%.6f, %.6f, %.6f)\n",
        //         speedX, speedY, speedZ, mCurrentPoseX, mCurrentPoseY, mCurrentPoseZ
        //     );
        // }

        //mDrive.drive(new ChassisSpeeds(speedX, speedY, speedZ));

        mDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
            speedX,
            speedY,
            speedZ,
            mDrive.getGyroscopeRotation()
            )
        );
        System.out.print("Angle: " + mDrive.getGyroscopeRotation().getDegrees() + " ");
        System.out.print("Delta: " + Math.abs(mDeltaZ));
        System.out.print("SetPoint " + mDeltaZ + " ");
        System.out.println();


    }

    @Override
    public boolean isFinished() {
        mCurrentPoseX = mDrive.getPose().getX();
        mCurrentPoseY = mDrive.getPose().getY();
        mCurrentPoseZ = mDrive.getGyroscopeRotation().getDegrees();

        boolean acheivedX = Math.abs(mDeltaX - mCurrentPoseX) < DriveConstants.kLinearDriveTranslationEpsilon;
        boolean acheivedY = Math.abs(mDeltaY - mCurrentPoseY) < DriveConstants.kLinearDriveTranslationEpsilon;
        boolean acheivedZ = Math.abs(mDeltaZ - mCurrentPoseZ) < DriveConstants.kLinearDriveRotationEpsilon;


        if(acheivedX && acheivedY  && acheivedZ) {
            System.out.println("Linear Drive Command Setpoint reached");
        }

        return acheivedX && acheivedY && acheivedZ;
    }
    @Override
    public void end(boolean interrupted) {
        System.out.println("Linear Drive Command finished (Interupted: "+ interrupted +")");
        mDrive.drive(new ChassisSpeeds());
    }
}