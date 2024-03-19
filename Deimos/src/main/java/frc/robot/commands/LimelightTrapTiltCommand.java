package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LampController;
import frc.robot.subsystems.DriveSubsystem;

public class LimelightTrapTiltCommand extends Command {
    private DriveSubsystem mDrive;
    private DriverStation mDriverStation;

    private ProfiledPIDController mPidHorizontalTilt;
    private ProfiledPIDController mPidHorizontalDrive;
    private ProfiledPIDController mPidVerticalDrive;


    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private TrapezoidProfile.Constraints constraints;

    private double horizontalTiltEpsilon = .1;
    private double horizontalDriveEpsilon = .1;
    private double verticalDriveEpsilon = .1;

    private LampController mLampController = LampController.getInstance();


    private double mPidHorizontalTiltGoal;
    private double mPidHorizontalDriveGoal;
    private double mPidVerticalDriveGoal;
    

    
    public LimelightTrapTiltCommand(DriveSubsystem mDriveSubsystem) {
        constraints = new TrapezoidProfile.Constraints(1, 1);
        this.mDrive = mDriveSubsystem;
        addRequirements(mDriveSubsystem);

    }   




    @Override
    public void initialize() {
        //start pid
        mPidHorizontalTilt = new ProfiledPIDController(.4,0,.1, constraints);
        mPidHorizontalDrive = new ProfiledPIDController(.4,0,.1, constraints);
        mPidVerticalDrive = new ProfiledPIDController(.4,0,.1, constraints);

        //table.getEntry("priorityid").setInteger(11);

        mPidHorizontalTiltGoal = 0; //angle
        mPidHorizontalDriveGoal = 0; //not an angle
        mPidVerticalDriveGoal = 16; //change to what the value actually is when the limelight measures it (angle)
        
    }

    @Override
    public void execute() {
        //run pid
        if(table.getEntry("tid").getInteger(0)!=-1){



            double xTilt  =  table.getEntry("tx").getDouble(0.0); //Angle measurement
            double horizontalTiltSpeed = mPidHorizontalTilt.calculate(xTilt/20, mPidHorizontalTiltGoal);

            double xDrive  =  table.getEntry("txnc").getDouble(0.0); //it's measured by pixels not meters or angle so might need to be converted
            double horizontalDriveSpeed = mPidHorizontalDrive.calculate(xDrive/20, mPidHorizontalTiltGoal);

            double yDrive  =  table.getEntry("ty").getDouble(0.0); // Angle measurement 
            double verticalDriveSpeed = -mPidVerticalDrive.calculate(yDrive, mPidVerticalDriveGoal);




            System.out.println("Horizontal Tilt Speed: " + horizontalTiltSpeed);
            System.out.println("Horizontal Drive Speed: " + horizontalDriveSpeed);
            System.out.println("Vertical Tilt Speed: " + verticalDriveSpeed);

            mDrive.drive(new ChassisSpeeds(horizontalDriveSpeed,0,horizontalTiltSpeed * Math.PI)); //might ned to change drivespeeds into meters
        }
    }
 
    @Override
    public boolean isFinished() {
        
        if (table.getEntry("tid").getInteger(0)==-1) {
            mDrive.drive(new ChassisSpeeds());

            System.out.println("Tilt Cancelled, no tag found ");
            return true;
        }
        if(Math.abs(mPidHorizontalTilt.getPositionError())<horizontalTiltEpsilon && Math.abs(mPidHorizontalDrive.getPositionError())<horizontalDriveEpsilon && Math.abs(mPidVerticalDrive.getPositionError())<verticalDriveEpsilon){
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







