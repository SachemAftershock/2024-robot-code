package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import org.opencv.core.Mat;

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
    private TrapezoidProfile.Constraints constraints;
    private double tiltEpsilon=.1;
    public LimelightTiltCommand(DriveSubsystem mDriveSubsystem) {
        constraints = new TrapezoidProfile.Constraints(1, 1);
        this.mDrive = mDriveSubsystem;
    }   

    @Override
    public void initialize() {
        //start pid
        mPidTilt = new ProfiledPIDController(.4,0,.1, constraints);

    }

    @Override
    public void execute() {
        //run pid
        double[] pose = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        if(table.getEntry("tid").getInteger(0)!=-1){
            double x  =  table.getEntry("tx").getDouble(0.0);
            double speed = -mPidTilt.calculate(x/20,0);
            System.out.println(speed);
            mDrive.drive(new ChassisSpeeds(0,0,speed));
        }
        mPidTilt.setGoal(0);

    }

    @Override
    public boolean isFinished() {
        
        if (table.getEntry("tid").getInteger(0)==-1) {
            mDrive.drive(new ChassisSpeeds());

            System.out.println("Tilt Cancelled, no tag found ");
            return true;
        }
        if(Math.abs(mPidTilt.getPositionError())<tiltEpsilon){
            mDrive.drive(new ChassisSpeeds());
            System.out.println("Tilt finished");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.drive(new ChassisSpeeds());
        System.out.println("end");
    }
}







