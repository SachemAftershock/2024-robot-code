package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LampController;
import frc.robot.subsystems.DriveSubsystem;

public class LimelightTiltCommand extends Command {
    private DriveSubsystem mDrive;
    private DriverStation mDriverStation;
    private ProfiledPIDController mPidTilt;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private TrapezoidProfile.Constraints constraints;
    private double tiltEpsilon = .1;
    private LampController mLampController = LampController.getInstance();
    private double mPIDGoal;
    

    
    public LimelightTiltCommand(DriveSubsystem mDriveSubsystem) {
        constraints = new TrapezoidProfile.Constraints(1, 1);
        this.mDrive = mDriveSubsystem;
        addRequirements(mDriveSubsystem);
    }   




    @Override
    public void initialize() {
        //start pid
        mPidTilt = new ProfiledPIDController(.4,0,.1, constraints);
        //table.getEntry("priorityid").setInteger(7);
        mPIDGoal = 0;
        
    }

    @Override
    public void execute() {
        //run pid
        if(table.getEntry("tid").getInteger(0)!=-1){
            double x  =  table.getEntry("tx").getDouble(0.0);
            double speed = mPidTilt.calculate(x/20, mPIDGoal);
            System.out.println(speed);
            mDrive.drive(new ChassisSpeeds(0,0,speed * Math.PI));
        }
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
            
			mLampController.setPulse(2, 0.2, 0.2, 0.8, true);
          
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







