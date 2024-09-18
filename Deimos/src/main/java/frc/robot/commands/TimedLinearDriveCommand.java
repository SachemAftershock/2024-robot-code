package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.PID;
import frc.robot.Constants.DriveConstants.CardinalDirection;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;


public class TimedLinearDriveCommand extends Command {

    private DriveSubsystem mDrive;
    private double mLinearSetpoint;
    private PID mPid;
    private double mCurrentPose; 
    private CardinalDirection mDirection;
    private Timer mTimer;
    private double speed;
    private boolean negative = false;
    private double setPointToTime;
    private double speedInMetersPerSecond,percentOfSlowTime,slowSpeedInMetersPerSecond;
    private double sign;
    //Field Relative : Y direction is horizontal, X direction is downfield 

    public TimedLinearDriveCommand(DriveSubsystem drive, double distanceInMeters, double speed, CardinalDirection direction) {
        mDrive = drive;
        mLinearSetpoint = distanceInMeters;
        mDirection = direction;
        speedInMetersPerSecond=speed;
        mPid = new PID();
        addRequirements(mDrive);
        mTimer = new Timer();
    }

    @Override
    public void initialize() {
        negative = mLinearSetpoint<=0;
        sign = negative ? -1 : 1;
        //desiredtime = desiredmovement/(actual moveemnt/actual time)
        double actualmovement=2.169; //85to86 incheas
        //setPointToTime=mLinearSetpoint/(actualmovement/(Math.abs(mLinearSetpoint)/speedInMetersPerSecond));
        //2/(2.169/(2/1))
        
        setPointToTime = (Math.abs(mLinearSetpoint)/speedInMetersPerSecond)/1.0845;
        
        mTimer.start();
        
        
        
        // mCurrentPose = 0.0;

        // if(mDirection == CardinalDirection.eY) {
        //     mLinearSetpoint += mDrive.getPose().getY();
        // } else {
        //     mLinearSetpoint += mDrive.getPose().getX();
        // }

        // mPid.start(DriveConstants.kDriveLinearGains); //TODO: tune pid values
        // System.out.println("Linear Drive Command started : Current Pose --> " + mDrive.getPose() + " Setpoint " + mLinearSetpoint);
    }

    @Override
    public void execute() {

        /**
         * Y direction is true
         * X direction is false
         */
        boolean yDirection = false;

        if(mDirection == CardinalDirection.eY) {
            // mCurrentPose = mDrive.getPose().getY();
            yDirection = true;
        } else {
            // mCurrentPose = mDrive.getPose().getX();
            yDirection = false;
        }

        //catch before the isfinished if its done
        if(mTimer.get()>setPointToTime){
            mDrive.drive(new ChassisSpeeds());
        }else{
            speed = speedInMetersPerSecond;
        }

        if(negative){
            speed = -speed;
        }

        if(yDirection) {
            mDrive.drive(new ChassisSpeeds(0, speed,0));//,  Math.toRadians(7.0*sign)));
        } else {
            mDrive.drive(new ChassisSpeeds(speed, 0, negative?Math.toRadians(7.0):0));
        }
    }

    @Override
    public boolean isFinished() {
        return mTimer.get()>setPointToTime;
    }
    @Override
    public void end(boolean interrupted) {
        mTimer.stop();
        mDrive.drive(new ChassisSpeeds());
    }
}