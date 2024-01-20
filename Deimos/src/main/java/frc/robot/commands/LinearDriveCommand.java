package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.PID;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.CardinalDirection;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


public class LinearDriveCommand extends Command {

    private DriveSubsystem mDrive;
    private double mLinearSetpoint;
    private PID mPid;
    private double mCurrentPose; 
    private CardinalDirection mDirection;
    private double prevSpeed;
    private TrapezoidProfile.Constraints m_constraints;
    private ProfiledPIDController m_controller;
    
    private int counter;
    //Field Relative : Y direction is horizontal, X direction is downfield 

    public LinearDriveCommand(DriveSubsystem drive, double setpoint, CardinalDirection direction) {
        mDrive = drive;
        mLinearSetpoint = setpoint;
        mDirection = direction;
        //mPid = new PID();

        m_constraints = new TrapezoidProfile.Constraints(DriveConstants.kMaxVelocityMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared);
        m_controller = new ProfiledPIDController(DriveConstants.kDriveLinearGains[0], DriveConstants.kDriveLinearGains[1], DriveConstants.kDriveLinearGains[2], m_constraints, DriveConstants.kDt);

        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        
        mCurrentPose = 0.0;
        //prevSpeed = 0.0;

        if(mDirection == CardinalDirection.eY) {
            mCurrentPose = mDrive.getPose().getY();
            mLinearSetpoint += mDrive.getPose().getY();
        } else {
            mCurrentPose = mDrive.getPose().getX();
            mLinearSetpoint += mDrive.getPose().getX();
        }

        m_controller.reset(mCurrentPose);

        //mPid.start(DriveConstants.kDriveLinearGains); //TODO: tune pid values
        System.out.println("Linear Drive Command started : Current Pose --> " + mDrive.getPose() + " Setpoint " + mLinearSetpoint);
        m_controller.setGoal(mLinearSetpoint);
        counter = 0;

        //mDrive.drive(new ChassisSpeeds(0, 0, 0));

    }

    @Override
    public void execute() {

        /**
         * Y direction is true
         * X direction is false
         */
        boolean direction = false;

        if(mDirection == CardinalDirection.eY) {
            mCurrentPose = mDrive.getPose().getY();
            direction = true;
        } else {
            mCurrentPose = mDrive.getPose().getX();
            direction = false;
        }
		// double speed = Math.sqrt(Math.abs(mPid.update(mCurrentPose, mLinearSetpoint)) 
        //     * DriveConstants.kMaxVelocityMetersPerSecond);

        // if (Math.abs(prevSpeed - speed) > 0.2) {
        //     if()
        //      speed = prevSpeed + 4.0; // Accelerating in positive direction
        // }

        // prevSpeed = speed;

        // if(Math.abs(speed) > 0.5){
        //     speed = 0.5;
        // }
        
        //delete
        //System.out.println(speed);

        double speed = m_controller.calculate(mCurrentPose);
        counter++;
        if(counter > 15) {
            counter = 0;
            System.out.println("speed: " + speed + " CurrentPose: " + mCurrentPose + "Setpoint: " + mLinearSetpoint );
        }

        if(direction) {
            //System.out.println("X-direc" + speed);
            mDrive.drive(new ChassisSpeeds(0, speed, 0));
        } else {
            mDrive.drive(new ChassisSpeeds(speed, 0, 0));
        }

    }

    @Override
    public boolean isFinished() {
        double mCurrentPose;
        if(mDirection == CardinalDirection.eY) {
            mCurrentPose = mDrive.getPose().getY();
        } else {
            mCurrentPose = mDrive.getPose().getX();
        }

        //return( Math.abs(m_controller.getPositionError()) < DriveConstants.kLinearDriveEpsilon);

        return Math.abs(mLinearSetpoint - mCurrentPose) < DriveConstants.kLinearDriveEpsilon;
    }
    @Override
    public void end(boolean interrupted) {
        System.out.println("Linear Drive Command finished");
        mDrive.drive(new ChassisSpeeds());
    }
}