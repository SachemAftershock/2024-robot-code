package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.PID;
import frc.robot.Constants.DriveConstants;
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

    //Field Relative : Y direction is horizontal, X direction is downfield 

    public TimedLinearDriveCommand(DriveSubsystem drive, double distanceInMeters, CardinalDirection direction) {
        mDrive = drive;
        mLinearSetpoint = distanceInMeters;
        mDirection = direction;
        mPid = new PID();
        addRequirements(mDrive);
        mTimer = new Timer();
    }

    @Override
    public void initialize() {
        speedInMetersPerSecond=1;
        slowSpeedInMetersPerSecond=.5;
        percentOfSlowTime = .2;
        negative = mLinearSetpoint<=0;
        setPointToTime = 
        (slowSpeedInMetersPerSecond/Math.abs(mLinearSetpoint)*percentOfSlowTime) +
        (speedInMetersPerSecond/Math.abs(mLinearSetpoint)*(1-percentOfSlowTime));//amount of slow time plus amount of normal time
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
        }


        //if time is later than settime at beginning and less than slow time at end 
        if(mTimer.get()>(setPointToTime*(percentOfSlowTime/2))
        &&
        mTimer.get()<(setPointToTime-setPointToTime*(percentOfSlowTime/2))){
            speed =speedInMetersPerSecond;
        }

        //if time is at slowtime at beginning
        else if(mTimer.get()<=(setPointToTime*(percentOfSlowTime/2))){
            speed = slowSpeedInMetersPerSecond;
        }

        //if time is at slowtime at end
        else if(mTimer.get()>=(setPointToTime-setPointToTime*(percentOfSlowTime/2))){
            speed=slowSpeedInMetersPerSecond;
        }

        if(negative){
            speed = -speed;
        }

        if(yDirection) {
            mDrive.drive(new ChassisSpeeds(0, speed, 0));
        } else {
            mDrive.drive(new ChassisSpeeds(speed, 0, 0));
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