// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.lib.PID;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.DriveConstants.CardinalDirection;
// import frc.robot.subsystems.DriveSubsystem;
// import com.kauailabs.navx.frc.AHRS;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.Timer;


// public class LimelightCommand extends Command {

//     private DriveSubsystem mDrive;
//     private PID mPidx, mPidZ;
//     private double CurrentBotRotation, ZDistanceFromTag, TagXPositionInLimelight;
//     private double ZSetpoint = 2;
//     private double YawRotationOfTag; //Changes based on apriltag yaw
//     private double XSetpoint = 0;
//     private double zSpeed, xSpeed, rotSpeed;
//     private final AHRS ahrs = new AHRS(SPI.Port.kMXP, (byte) 200);
//     NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
//     private int targetID;
//     private double setAngle;

//     //Field Relative : Y direction is horizontal, X direction is downfield 

//     public LimelightCommand(DriveSubsystem drive) {
//         mDrive = drive;
//         mPidx = new PID();
//         mPidZ = new PID();
//         addRequirements(mDrive);
//     }

//     @Override
//     public void initialize() {
//         mPidx.start(DriveConstants.kDriveLinearGains); //TODO: tune pid values
//         mPidZ.start(DriveConstants.kDriveLinearGains); //TODO: tune pid values
//     }
//     public void setTarget(double angle, int id){
//         targetID=id;
//         setAngle = angle;
        
//     }
//     public boolean hasTargetID(){
//         if()
//         if(targetID==table.getEntry("tid").getDouble(0.0)){
//             return true;
//         }
//         return false;
//     }
//     //button sets angle parameter based on left right or center or amp, button also sets id as a paraeter
    
//     public double getPIDRot(){
//         double[] pose = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
//         YawRotationOfTag = pose[0];
//         if(hasTargetID()){

//             TagXPositionInLimelight = table.getEntry("tx").getDouble(0.0);
//             double speed = mPidZ.update(ZDistanceFromTag, ZSetpoint) 
//             * DriveConstants.kMaxVelocityMetersPerSecond;
//             return speed;
//         }else{
//             return 0;
//         }
//     }
//     public double getPIDX(){
//         if(hasTargetID()){
//             TagXPositionInLimelight = table.getEntry("tx").getDouble(0.0);
//             double speed = mPidZ.update(ZDistanceFromTag, ZSetpoint) 
//             * DriveConstants.kMaxVelocityMetersPerSecond;
//             return speed;
//         }else{
//             return 0;
//         }
//     }
//     public double getPIDZ(){
//         double[] pose = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
//         if(hasTargetID()){
//             ZDistanceFromTag = pose[2];
//             double speed = mPidZ.update(ZDistanceFromTag, ZSetpoint) 
//             * DriveConstants.kMaxVelocityMetersPerSecond;
//             return speed;
//         }else{
//             return 0;
//         }
//     }
//     @Override
//     public void execute() {
//         /*
//         NetworkTableEntry tx = table.getEntry("tx");
//         NetworkTableEntry ty = table.getEntry("ty");
//         double[] pose = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
//         // double ID = table.getEntry("tid").getInteger(0);
//         CurrentBotRotation = ahrs.getYaw(); //current rotation based on navx (ZYAW)
//         ZDistanceFromTag = pose[2]; //fill with limelight data based on distance from tag
//         YawRotationOfTag = pose[0]; //fill with limelight data of rotation of tag
//         TagXPositionInLimelight = tx.getDouble(0.0); //fill with limelight data of location of tag on x axis

// 		zSpeed = mPidZ.update(ZDistanceFromTag, ZSetpoint) 
//             * DriveConstants.kMaxVelocityMetersPerSecond;
//         xSpeed = mPidx.update(TagXPositionInLimelight, XSetpoint) 
//         * DriveConstants.kMaxVelocityMetersPerSecond;
//         rotSpeed = mPidRot.update(CurrentBotRotation, YawRotationOfTag) 
//         * DriveConstants.kMaxVelocityMetersPerSecond;
//         double time = Timer.getFPGATimestamp();

//         if(time+1<Timer.getFPGATimestamp()){
//             time = Timer.getFPGATimestamp();
//             System.out.println("XSpeed: "+ xSpeed);
//             System.out.println("ZSpeed: "+ zSpeed);
//             System.out.println("rotSpeed: "+ rotSpeed);
//         }
//         if(xSpeed>1){xSpeed=1;}
//          if(zSpeed>1){xSpeed=1;}
//           if(rotSpeed>1){xSpeed=1;}
//         mDrive.drive(new ChassisSpeeds(xSpeed*.3, zSpeed*.3, rotSpeed*.3));
//         */
//     }

//     @Override
//     public boolean isFinished() {
//         return (
//             Math.abs(mPidx.getError()) < DriveConstants.kLinearDriveEpsilon
//             &&Math.abs(mPidRot.getError()) < DriveConstants.kLinearDriveEpsilon
//             &&Math.abs(mPidZ.getError()) < DriveConstants.kLinearDriveEpsilon);
//     }
//     @Override
//     public void end(boolean interrupted) {
//         mDrive.drive(new ChassisSpeeds());
//     }
// }