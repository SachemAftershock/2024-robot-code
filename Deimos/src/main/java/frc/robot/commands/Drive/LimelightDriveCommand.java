package frc.robot.commands.Drive;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.PID;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.PID;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import com.kauailabs.navx.frc.AHRS;

/**
 * stuff. TODO documentation
 */
public class LimelightDriveCommand extends Command {
  private double CurrentBotRotation, ZDistanceFromTag, TagXPositionInLimelight;
  private double zSetpoint, xSetpoint, rotationSetpoint;
  private double YawRotationOfTag; // Changes based on apriltag yaw
  private double zSpeed, xSpeed, rotSpeed;
  DriveSubsystem mDriveSubsystem;
  // MedianFilter filter = new MedianFilter(50);
  double time = Timer.getFPGATimestamp();
  int i = 0;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  AHRS ahrs;
  ProfiledPIDController mPidx, mPidZ;
  PID mPidRot;

  /*
   * rotationSetpoint is in degrees
   */
  public LimelightDriveCommand(
      DriveSubsystem mDriveSubsystem,
      AHRS navx, ProfiledPIDController mPidx, ProfiledPIDController mPidZ, PID mPidRot,
      double xSetpoint, double zSetpoint, double rotationSetpoint) {
    addRequirements(mDriveSubsystem);
    this.ahrs = navx;
    this.mPidx = mPidx;
    this.mPidZ = mPidZ;
    this.mPidRot = mPidRot;
    this.mDriveSubsystem = mDriveSubsystem;
    this.xSetpoint = xSetpoint;
    this.zSetpoint = zSetpoint * -1.0; // Must negate if positive is away from apriltag
    this.rotationSetpoint = rotationSetpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // CommandScheduler.getInstance().schedule(lime);
    double[] pose = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    double ID = table.getEntry("tid").getInteger(0);
    double y = ty.getDouble(0.0);
    double x = tx.getDouble(0.0) / 12;
    double yaw = ahrs.getYaw(); // current rotation based on navx (ZYAW)
    if (ID != -1) {// figure out isfinished

      // All Values Tested
      // System.out.println("Pitch angle is " + pose[1]); //XY tilt in radians;
      // Boundaries: (-pi,pi)
      // System.out.println("Yaw angle of tag is " + pose[0] * 720/Math.PI); //XZ tilt
      // in radians; Boundaries; (-pi,pi)
      // System.out.println("ID is " + ID);
      // System.out.println("Robot Yaw: "+ -1 * yaw);
      // System.out.println("X value is " + x); //Right is positive; left is negative;
      // Boundaries: ~(-24,24)
      // System.out.println("Y value is " + y); //Up is positive; Down is negative;
      // Boundaries: around -10 to 10
      // System.out.println("Z value is " + (pose[2] * -1)); //Starts at 0 and
      // increases farther from the April Tag in meters

      // Z Distance TESTED
      ZDistanceFromTag = pose[2]; // negative distance away in meters
      // System.out.println(ZDistanceFromTag);
      zSpeed = mPidZ.calculate(ZDistanceFromTag, zSetpoint);
      // System.out.println(i + " Zspeed: " + zSpeed);

      // X Centering Tested for Trapezoiding
      TagXPositionInLimelight = x;
      // System.out.println(x);
      xSpeed = -1.0 * mPidx.calculate(TagXPositionInLimelight, xSetpoint);
      // System.out.println(i + " Xspeed:" + xSpeed);

      // Rotation Tested
      CurrentBotRotation = yaw;
      YawRotationOfTag = pose[0] * 180;
      rotSpeed = mPidRot.updateRotation(YawRotationOfTag, rotationSetpoint) / 2.0 / -1.0; // good luck
      // currently rotSpeed is clamped between -.5 and .5 for dividing by 2. it also
      // has to be multiplied
      // by -1 so that rotation clockwise is positive
      // TODO WEIRD BUG
      // YawRotationOfTag divided by 100 and clamped between -.5 and .5 gets the same
      // value as rotSpeed when all values are put through the updateRotation PID

      // +zSpeed is towards limelight
      // +xSpeed is towards right
      // +rotSpeed is clockwise
      /*
       * ChassisSpeeds(
       * 1. positive is towards apriltag,
       * 2. positive is towards right,
       * 3. positive is clockwise
       * )
       */
      ChassisSpeeds mChassisSpeeds = new ChassisSpeeds(zSpeed, xSpeed, rotSpeed);

      mDriveSubsystem.drive(mChassisSpeeds);

      if (i % 50 == 0) {
        // System.out.println(YawRotationOfTag);//when robot is on the left side, value
        // is negative, when robot is on right side, value is posiitive
        // System.out.println(CurrentBotRotation);// clockwise is positiv, counter
        // clockwise is negative
        // System.out.println(i+" rotSpeed: " + rotSpeed + " yawRotTag: " +
        // YawRotationOfTag); // rotspeed is motor
        System.out.println(mChassisSpeeds);

      }
      i++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (mPidRot.getError() < .1 && mPidZ.getPositionError() < .1 && mPidx.getPositionError() < .1) {
      // System.out.println("finished");
      return true;
    } else {
      return false;
    }
  }
}
