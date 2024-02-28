package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.PID;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

public class LimelightTiltCommand extends Command {

    private DriveSubsystem mDrive;
    private ProfiledPIDController mPidTilt;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private LimelightSubsystem mLimelightSubsystem;
    private TrapezoidProfile.Constraints constraints;
    private double tiltEpsilon=.1;
    public LimelightTiltCommand(DriveSubsystem mDrive) {
        constraints = new TrapezoidProfile.Constraints(DriveConstants.kMaxVelocityMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared);
        this.mLimelightSubsystem = mLimelightSubsystem;//todo: implement limelightsubsystem
        this.mDrive=mDrive;
        addRequirements(mDrive);
    }   

    @Override
    public void initialize() {
        //start pid
        mPidTilt = new ProfiledPIDController(DriveConstants.kDriveLinearGains[0], DriveConstants.kDriveLinearGains[1], DriveConstants.kDriveLinearGains[2], constraints);

    }

    @Override
    public void execute() {
        //run pid
        double[] pose = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        if(table.getEntry("tid").getInteger(0)!=-1){
            double x  =  table.getEntry("tx").getDouble(0.0);
            double speed = mPidTilt.calculate(x,0)* DriveConstants.kMaxVelocityMetersPerSecond;
            mDrive.drive(new ChassisSpeeds(0,0,speed));
        }
        mPidTilt.setGoal(0);

    }

    @Override
    public boolean isFinished() {
        if(mPidTilt.getPositionError()<tiltEpsilon){
            mDrive.drive(new ChassisSpeeds());
            System.out.println("Tilt finished");
            return true;
        }
        if (table.getEntry("tid").getInteger(0)==-1) {
            mDrive.drive(new ChassisSpeeds());

            System.out.println("Tilt Cancelled, no tag found ");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.drive(new ChassisSpeeds());

    }
}







