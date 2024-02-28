package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AftershockSubsystem;
import frc.lib.PID;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.CardinalDirection;
import frc.robot.subsystems.DriveSubsystem;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

public class LimelightSubsystem extends AftershockSubsystem {

	private static LimelightSubsystem mInstance;
private DriveSubsystem mDrive;
    private PID mPidx, mPidZ, mPidRot;
    private double CurrentBotRotation, ZDistanceFromTag, TagXPositionInLimelight;
    private double ZSetpoint = 2;
    private double YawRotationOfTag; //Changes based on apriltag yaw
    private double XSetpoint = 0;
    private double zSpeed, xSpeed, rotSpeed;
    private final AHRS ahrs = new AHRS(SPI.Port.kMXP, (byte) 200);
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    	
    private double tx;
    private double ty;
    private double tz;
    private double ta;
    private double[] pose;
    private int targetID;
    private double setAngle;

	private LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        pose = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        tx = pose[0];
        ty = pose[1];
        tz = pose[2];
        ta = table.getEntry("ta").getDouble(9999.9);


		//object declarations
	}
	
	@Override
	public void initialize() {
		//initializing methods
		mDrive = DriveSubsystem.getInstance();
        mPidx = new PID();
        mPidZ = new PID();
        mPidRot = new PID();
	}
	public void setTarget(double angle, int id){
        targetID=id;
        setAngle = angle;
        
    }
    
    public boolean hasTargetID(){
        if()
        if(targetID==table.getEntry("tid").getDouble(0.0)){
            return true;
        }
        return false;
    }
    //button sets angle parameter based on left right or center or amp, button also sets id as a paraeter
    
    public double getPIDRot(){
        double[] pose = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        YawRotationOfTag = pose[0];
        if(hasTargetID()){

            TagXPositionInLimelight = table.getEntry("tx").getDouble(0.0);
            double speed = mPidZ.update(ZDistanceFromTag, ZSetpoint) 
            * DriveConstants.kMaxVelocityMetersPerSecond;
            return speed;
        }else{
            return 0;
        }
    }
    public double getPIDX(){
        if(hasTargetID()){
            TagXPositionInLimelight = table.getEntry("tx").getDouble(0.0);
            double speed = mPidZ.update(ZDistanceFromTag, ZSetpoint) 
            * DriveConstants.kMaxVelocityMetersPerSecond;
            return speed;
        }else{
            return 0;
        }
    }
    public double getPIDZ(){
        double[] pose = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        if(hasTargetID()){
            ZDistanceFromTag = pose[2];
            double speed = mPidZ.update(ZDistanceFromTag, ZSetpoint) 
            * DriveConstants.kMaxVelocityMetersPerSecond;
            return speed;
        }else{
            return 0;
        }
    }



	@Override
	public void periodic() {
	}

	//Add Shuffle board calls here
	public void outputTelemetry() {

	}

	public synchronized static LimelightSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new LimelightSubsystem();
		}
		return mInstance;
	}
}