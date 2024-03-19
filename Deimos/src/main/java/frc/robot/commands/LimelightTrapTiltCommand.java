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

public class LimelightTrapTiltCommand extends Command {
    private DriveSubsystem mDrive;
    private DriverStation mDriverStation;
    private ProfiledPIDController mPidTilt;
     private ProfiledPIDController mPidLeftRight;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private TrapezoidProfile.Constraints constraints;
    private double horTiltEpsilon = .1;
    private double verTiltEpsilon = .1;
    private double distanceEpsilon = .1;
    private LampController mLampController = LampController.getInstance();
    private double mPIDHorAngleGoal;
    private double mPIDDistanceGoal;
    private double mPIDVertAngleGoal;
    

    
    public LimelightTrapTiltCommand(DriveSubsystem mDriveSubsystem) {
        constraints = new TrapezoidProfile.Constraints(1, 1);
        this.mDrive = mDriveSubsystem;
    }   




    @Override
    public void initialize() {
        //start pid
        mPidTilt = new ProfiledPIDController(.4,0,.1, constraints);
        mPidLeftRight = new ProfiledPIDController(4,0, .1, constraints);
        table.getEntry("priorityid").setInteger(11);
        mPIDDistanceGoal = 0;
        mPIDHorAngleGoal = 0;
        mPIDVertAngleGoal = 0;//get from limelight
    }

    @Override
    public void execute() {
        //run pid
        if(table.getEntry("tid").getInteger(0)!=-1){
            double horizontalDistance = table.getEntry("txnc").getDouble(0);
            double horizontalAngle  =  table.getEntry("tx").getDouble(0.0);
            double verticalAngle  =  table.getEntry("ty").getDouble(0.0);

            double vertAngleSpeed = -mPidTilt.calculate(verticalAngle/20, mPIDVertAngleGoal);
            double horAngleSpeed = -mPidTilt.calculate(horizontalAngle/20, mPIDHorAngleGoal);
            double distanceSpeed = mPidLeftRight.calculate(horizontalDistance, mPIDDistanceGoal);
            


            System.out.println("Angle Speed: " + vertAngleSpeed);
            System.out.println("Distance Speed: " + distanceSpeed);
            mDrive.drive(new ChassisSpeeds(distanceSpeed, horAngleSpeed,-vertAngleSpeed * Math.PI));
        }
    }
 
    @Override
    public boolean isFinished() {
        
        if (table.getEntry("tid").getInteger(0)==-1) {
            mDrive.drive(new ChassisSpeeds());

            System.out.println("Tilt/Drive Cancelled, no tag found ");
            return true;
        }
        if(Math.abs(mPidTilt.getPositionError()) < horTiltEpsilon && Math.abs(mPidLeftRight.getPositionError()) < distanceEpsilon){
            mDrive.drive(new ChassisSpeeds());
            System.out.println("Tilt/Drive finished");
            
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