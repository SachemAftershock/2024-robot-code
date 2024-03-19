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
    private double tiltEpsilon = 1.0;
    private LampController mLampController = LampController.getInstance();
    private boolean mLampTriggered = false;
    private Alliance AllianceColor = null;
    private int LocationNumber = 0;
    private double mPIDGoal;
    private int mTag;
    private double mAngle;
    private double mMaxTime = 2;
    private double mStartTime;
    

    
    public LimelightTiltCommand(DriveSubsystem mDriveSubsystem, int Tag, double Angle) {
        mTag = Tag;
        mAngle = Angle;
        constraints = new TrapezoidProfile.Constraints(1, 1);
        this.mDrive = mDriveSubsystem;
        mStartTime = Timer.getFPGATimestamp();
        addRequirements(mDriveSubsystem);
    }   

    public LimelightTiltCommand(DriveSubsystem mDriveSubsystem) {
        mTag = -1;
        mAngle = 0;
        constraints = new TrapezoidProfile.Constraints(1, 1);
        this.mDrive = mDriveSubsystem;
        addRequirements(mDriveSubsystem);
    }   



    @Override
    public void initialize() {
        //start pid
        mPidTilt = new ProfiledPIDController(.05,0,.1, constraints);
        table.getEntry("priorityid").setInteger(mTag);
        mPIDGoal = mAngle;
        if (mTag == -1){
            System.out.println("No priority tag set");
        }
    }

    @Override
    public void execute() {
        //run pid
        if(table.getEntry("tid").getInteger(0)!=-1){
            double x  =  table.getEntry("tx").getDouble(0.0);
            double speed = -mPidTilt.calculate(x, mPIDGoal);
            System.out.println(speed);
            mDrive.drive(new ChassisSpeeds(0,0,speed * Math.PI));
        }
    }
 
    int counter=0;
    @Override
    public boolean isFinished() {

        // If the command lasts longer than 2 seconds, kill it
        if (Timer.getFPGATimestamp() - mStartTime > mMaxTime) {
            return true;
        }
        
        if (table.getEntry("tid").getInteger(0)==-1) {
            mDrive.drive(new ChassisSpeeds());

            counter++;
            if (counter % 50 == 0)
                System.out.println("Tilt Cancelled, no tag found ");
            return true;
        }

        //silyboi
        
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






