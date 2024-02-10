// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.AftershockXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.CardinalDirection;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.FollowTrajectoryCommandFactory;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.commands.RotateDriveCommand;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterAngleState;
import frc.robot.commands.LinearDriveCommand;
import frc.robot.commands.ManualDriveCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();
  private IntakeSubsystem mIntakeSubsystem = IntakeSubsystem.getInstance();
  private ShooterSubsystem mShooterSubsystem = ShooterSubsystem.getInstance();
  
  private final AftershockXboxController mControllerPrimary = new AftershockXboxController(0);
  private final Joystick mControllerSecondary = new Joystick(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    mDriveSubsystem.setDefaultCommand(new ManualDriveCommand(
            mDriveSubsystem,
            () -> -modifyAxis(mControllerSecondary.getY()) * DriveConstants.kMaxVelocityMetersPerSecond * 0.7,
            () -> -modifyAxis(mControllerSecondary.getX()) * DriveConstants.kMaxVelocityMetersPerSecond * 0.7,//() -> -modifyAxis(mControllerPrimary.getLeftX()) * DriveConstants.kMaxVelocityMetersPerSecond * 0.7,

            () ->-modifyAxis(mControllerSecondary.getTwist()) * DriveConstants.kMaxAngularVelocityRadiansPerSecond * 0.3//() -> -modifyAxis(mControllerSecondary.getTwist()) * DriveConstants.kMaxAngularVelocityRadiansPerSecond * 0.3

    ));
    
  }

  public void initialize() {
    mDriveSubsystem.initialize();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //ROLLERS
    // RB for something
    Trigger IntakeRollerIngestTrigger = new Trigger(() -> mControllerPrimary.getRightBumperPressed());

    IntakeRollerIngestTrigger.onTrue(new InstantCommand(() -> { 
      mIntakeSubsystem.setRollerMotorSpeed(-0.25);
    })).onFalse(new InstantCommand(() -> mIntakeSubsystem.setRollerMotorSpeed(0.0)));

    //ARM
    // A for retracted
    Trigger IntakeArmIngestTrigger = new Trigger(() -> mControllerPrimary.getAButton());

    IntakeArmIngestTrigger.onTrue(new InstantCommand(() -> { 
      mIntakeSubsystem.setDesiredIntakeState(IntakeState.eRetracted);
    }));

    // Y for deployed
    Trigger negIntakeArmIngestTrigger = new Trigger(() -> mControllerPrimary.getYButton());

    negIntakeArmIngestTrigger.onTrue(new InstantCommand(() -> { 
      mIntakeSubsystem.setDesiredIntakeState(IntakeState.eDeployed);
    }));

    // LT and RT for shooter motors
    // LT is a bit left, RT is a bit right, both is straight forward
    Trigger ShooterMotorTrigger = new Trigger(() -> {
      return mControllerPrimary.getLeftTriggerAxis() > .05
        || mControllerPrimary.getRightTriggerAxis() > .05;
    });

    ShooterMotorTrigger
      .onTrue(new InstantCommand(() -> {
        boolean LTPushed = mControllerPrimary.getLeftTriggerAxis() > .05;
        boolean RTPushed = mControllerPrimary.getRightTriggerAxis() > .05;
        boolean bothPushed = LTPushed && RTPushed;
        if (LTPushed && RTPushed) {
          mShooterSubsystem.spinShooterMotors(1, 1);
        } else if (LTPushed) {// TODO make sure this is left
          mShooterSubsystem.spinShooterMotors(.5,1); 
        } else if (RTPushed) {
          mShooterSubsystem.spinShooterMotors(1,.4);
        }
      })).onFalse(new InstantCommand(()-> {
        mShooterSubsystem.spinShooterMotors(0, 0);
    }));

    // Left joystick Y to change angle of shooter
    Trigger AngleShootMotorTrigger = new Trigger(()->{
      return Math.abs(mControllerPrimary.getLeftY()) > .1;
    });
    AngleShootMotorTrigger
      .onTrue(new InstantCommand(()->{
        mShooterSubsystem.setAngleShooterMotorSpeed(mControllerPrimary.getLeftY());
      })).onFalse(new InstantCommand(()->{
        mShooterSubsystem.setAngleShooterMotorSpeed(0);
    }));

    // X to aim at amp, otherwise aim at speaker
    Trigger AngleShootMotorPIDTrigger = new Trigger(()->{
      return mControllerPrimary.getXButton();
    });
    AngleShootMotorPIDTrigger.whileTrue(new InstantCommand(()->{
      mShooterSubsystem.setDesiredShooterAngleState(ShooterAngleState.eAmp);
      mShooterSubsystem.runShooterAnglePID();
    }).repeatedly()).whileFalse(new InstantCommand(()->{
      mShooterSubsystem.setDesiredShooterAngleState(ShooterAngleState.eSpeaker);
      mShooterSubsystem.runShooterAnglePID();
    }).repeatedly());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // TrajectoryConfig config = new TrajectoryConfig(
    //   DriveConstants.kMaxVelocityMetersPerSecond * 0.3, 
    //   DriveConstants.kMaxAccelerationMetersPerSecondSquared
    // );

    // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    //   new Pose2d(), 
    //   List.of(
    //     new Translation2d(2.0, -0.5)
    //     //new Translation2d(0, 1.2)//,
    //     //new Translation2d(1, 1.5)
    //     //new Translation2d(2.2,0)
    //   ),
    //   new Pose2d(1.0, -0.4, new Rotation2d()), 
    //   config
    // );

    // Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
    //   new Pose2d(), 
    //   List.of( 
    //     new Translation2d(0.5, 1.5),
    //     new Translation2d(1.0, 1.5),
    //     new Translation2d(1.5, 1.5)
    //   ),
    //   new Pose2d(1.5, 1.5, new Rotation2d()),
    //   config
    // );
    //return new RotateDriveCommand(mDriveSubsystem, 90);
    //mDriveSubsystem.zeroGyroscope();

    
   /**  return new DelayCommand(1.0).andThen
    (new LinearDriveCommand(mDriveSubsystem, 2.5, 0.0, 0.0)).andThen
    (new LinearDriveCommand(mDriveSubsystem, 0.0, 2.5, 0.0)).andThen
    (new LinearDriveCommand(mDriveSubsystem, -2.5, 0.0, 0.0)).andThen
    (new LinearDriveCommand(mDriveSubsystem, 0.0, -2.5, 0.0)); //was 2.0**/
    

    return new DelayCommand(1.0).andThen
    (new LinearDriveCommand(mDriveSubsystem, 1.0, 1.0, 360.0)).andThen
      ((new LinearDriveCommand(mDriveSubsystem, -1.0, 1.0, 360.0)).alongWith
      (new RetractIntakeCommand(mIntakeSubsystem))).andThen
    (new LinearDriveCommand(mDriveSubsystem, -1.0, -1.0, 360.0)).andThen
    (new LinearDriveCommand(mDriveSubsystem, 1.0, -1.0, 360.0));

    // return new  DelayCommand(1.0).andThen
    // (new LinearDriveCommand(mDriveSubsystem, 1.0, 1.0, 0.0)).andThen
    // (new LinearDriveCommand(mDriveSubsystem, 1.0, -2.0, 90.0)).andThen
    // (new LinearDriveCommand(mDriveSubsystem, 1.0, 2.0, -90.0)).andThen
    // (new LinearDriveCommand(mDriveSubsystem, 1.0, -1.0, 180)).andThen
    // (new LinearDriveCommand(mDriveSubsystem, 1.0, 0.0, 45.0)).andThen
    // (new LinearDriveCommand(mDriveSubsystem, 1.0, 0.0, 45.0));



    //return (new LinearDriveCommand(mDriveSubsystem, 2.5, 0.0, 0.0)).raceWith(new RotateDriveCommand(mDriveSubsystem, 90));
    
    /**return new DelayCommand(1.0).andThen
    (new LinearDriveCommand(mDriveSubsystem, 2.5, 0.0, 0.0)).andThen
    //(new DelayCommand(1.0)).andThen
    (new LinearDriveCommand(mDriveSubsystem, 0.0, 2.5, 0.0))
    .raceWith(new RotateDriveCommand(mDriveSubsystem, 0)).andThen

    (new LinearDriveCommand(mDriveSubsystem, -2.5, 0.0, 0.0)).andThen
    (new LinearDriveCommand(mDriveSubsystem, 0.0, -2.5, 0.0));**/
    
    
    /**return new SequentialCommandGroup(
      new LinearDriveCommand(mDriveSubsystem, 2, CardinalDirection.eX),
      new RotateDriveCommand(mDriveSubsystem, 180),
      new LinearDriveCommand(mDriveSubsystem, 2, CardinalDirection.eX)
      
    //FollowTrajectoryCommandFactory.generateCommand(mDriveSubsystem, trajectory),
       //new RotateDriveCommand(mDriveSubsystem, 90),
       //FollowTrajectoryCommandFactory.generateCommand(mDriveSubsystem, trajectory2),
       //new RotateDriveCommand(mDriveSubsystem, -30)

      
     );**/
     
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, DriveConstants.kDriveControllerDeadband);

    // Square the axis
    if(DriveConstants.kSquareAxis) {
      value = Math.copySign(value * value, value);
    }

    return value;
  }

  public void resetIntakeCalibration(){
		mIntakeSubsystem.resetCalibration();
	}
}

