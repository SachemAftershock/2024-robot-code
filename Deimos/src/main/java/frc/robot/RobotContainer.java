// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ClimberConstants.kClimberMotorSpeed;
import static frc.robot.Constants.ShooterConstants.kShootMotorShootingVelocity;

import com.fasterxml.jackson.databind.util.PrimitiveArrayBuilder;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem.climberMotorToSpinEnum;
import frc.robot.subsystems.IntakeSubsystem.IntakeArmPositionEnum;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.FollowTrajectoryCommandFactory;
import frc.robot.commands.IngestNoteCommand;
import frc.robot.commands.EjectNoteCommand;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.commands.RotateDriveCommand;
import frc.robot.commands.ShooterMotorsOffCommand;
import frc.robot.commands.ShooterMotorsToSpeakerSpeedCommand;
import frc.robot.commands.ShooterStageToNoteLoadAngleCommand;
import frc.robot.commands.ShooterStageToSpeakerAngleCommand;
import frc.robot.commands.AutoCommands.AutoAmpScoreSequence;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterAngleState;
import frc.robot.commands.LinearDriveCommand;
import frc.robot.commands.ManualAmpScoreCommand;
import frc.robot.commands.ManualDriveCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LimelightManagerSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private boolean mIsMappedForShooterNotClimber = true;
  public static boolean isScoreTriggered = false;

  // The robot's subsystems and commands are defined here...
  private DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();
  private ClimberSubsystem mClimberSubsystem = ClimberSubsystem.getInstance();
  private IntakeSubsystem mIntakeSubsystem = IntakeSubsystem.getInstance();
  private ShooterSubsystem mShooterSubsystem = ShooterSubsystem.getInstance();
  
  private final AftershockXboxController mControllerTertiary = new AftershockXboxController(0);
  private final Joystick mControllerPrimary = new Joystick(1);
  private final Joystick mControllerSecondary = new Joystick(2);
  
  private Command sequenceDeployIngestRetractEject = new SequentialCommandGroup(
    (new DelayCommand(0.1)).andThen
    (new DeployIntakeCommand(mIntakeSubsystem)).andThen
    //(new DelayCommand(0.2)).andThen
    (new IngestNoteCommand(mIntakeSubsystem)).andThen
    //(new ShooterStageToNoteLoadAngleCommand(mShooterSubsystem)).andThen
    //(new DelayCommand(0.2)).andThen
    (new RetractIntakeCommand(mIntakeSubsystem)).andThen
    //(new DelayCommand(0.2)).andThen
    (new EjectNoteCommand(mIntakeSubsystem))
  ); 

  private Command sequenceDeployIngestRetract = new SequentialCommandGroup(
    (new DelayCommand(0.05)).andThen
    (new DeployIntakeCommand(mIntakeSubsystem)).andThen
    (new DelayCommand(0.1)).andThen
    (new IngestNoteCommand(mIntakeSubsystem)).andThen
    (new DelayCommand(0.35)).andThen
    //(new ShooterStageToNoteLoadAngleCommand(mShooterSubsystem)).andThen
    //(new DelayCommand(0.2)).andThen
    (new RetractIntakeCommand(mIntakeSubsystem))
  ); 

  private Command sequenceStopRollersAndRetract = new SequentialCommandGroup(
    // (new DelayCommand(0.1)).andThen
    (new InstantCommand(() -> mIntakeSubsystem.setRollerMotorSpeed(0.0))).andThen
    //(new DelayCommand(0.2)).andThen
    (new RetractIntakeCommand(mIntakeSubsystem))
  ); 

  // B to suck a note, then retract, then LT to go to speaker position and spin motors
  // RT is ignored until ...
  private Command sequenceArmToFire = new SequentialCommandGroup(
    (new DelayCommand(0.1)).andThen
    (new ShooterStageToNoteLoadAngleCommand(mShooterSubsystem)).andThen
    (new DelayCommand(0.2)).andThen
    (new RetractIntakeCommand(mIntakeSubsystem)).andThen
    (new DelayCommand(0.2)).andThen
    (new ShooterMotorsToSpeakerSpeedCommand(mShooterSubsystem)).andThen
    (new DelayCommand(0.2)).andThen
//    (new ShooterMotorsOffsetsPerLateralSpeakerAngleCommand(mShooterSubsystem)).andThen
//    (new DelayCommand(0.2)).andThen
    (new ShooterStageToSpeakerAngleCommand(mShooterSubsystem)).andThen
    (new DelayCommand(0.2)).andThen
    (new InstantCommand(() -> { mArmedToFire = true; }))
  );

  private Command sequenceDisarm = new SequentialCommandGroup(
    (new DelayCommand(0.1)).andThen
   (new ShooterMotorsOffCommand(mShooterSubsystem)).andThen
   (new DelayCommand(0.2)).andThen
    (new InstantCommand(() -> { mArmedToFire = false; }))
  );

  private Command sequenceFireNote = new SequentialCommandGroup(
    (new DelayCommand(0.1)).andThen
    (new EjectNoteCommand(mIntakeSubsystem))  //.andThen
//    (new ShooterMotorsOffCommand(mShooterSubsystem)).andThen
//    (new DelayCommand(0.2))
  ); 

  private boolean mArmedToFire = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    mDriveSubsystem.setDefaultCommand(new ManualDriveCommand(
            mDriveSubsystem,
            () -> -modifyAxis(mControllerPrimary.getY()) * DriveConstants.kMaxVelocityMetersPerSecond * 0.7,
            () -> -modifyAxis(mControllerPrimary.getX()) * DriveConstants.kMaxVelocityMetersPerSecond * 0.7,//() -> -modifyAxis(mControllerPrimary.getLeftX()) * DriveConstants.kMaxVelocityMetersPerSecond * 0.7,

            () ->-modifyAxis(mControllerSecondary.getTwist()) * DriveConstants.kMaxAngularVelocityRadiansPerSecond * 0.3//() -> -modifyAxis(mControllerSecondary.getTwist()) * DriveConstants.kMaxAngularVelocityRadiansPerSecond * 0.3
    ));
    // mIntakeSubsystem.setDefaultCommand(new ManualIntakeArm(
    //         mIntakeSubsystem,
    //         () -> -modifyAxis(mControllerPrimary.getY()) * DriveConstants.kMaxVelocityMetersPerSecond * 0.7),
    //         () -> -modifyAxis(mControllerPrimary.getY()) * DriveConstants.kMaxVelocityMetersPerSecond * 0.7);
    
  }

  public void initialize() {
    mDriveSubsystem.initialize();
    LimelightManagerSubsystem.getInstance().initialize();;

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //INTAKE ROLLERS (Manual)

    Trigger IntakeRollerIngestTrigger 
      = new Trigger(() -> mControllerTertiary.getRightBumperPressed());
    IntakeRollerIngestTrigger
      .onTrue(new InstantCommand(() -> { mIntakeSubsystem.setRollerMotorSpeed(-0.25); } ));

    Trigger IntakeRollerStopIngestTrigger 
      = new Trigger(() -> mControllerTertiary.getRightBumperReleased());
    IntakeRollerStopIngestTrigger
      .onTrue(new InstantCommand(() -> { mIntakeSubsystem.setRollerMotorSpeed(0.0); } ));

    Trigger IntakeRollerEjectTrigger 
      = new Trigger(() -> mControllerTertiary.getLeftBumperPressed());
    IntakeRollerEjectTrigger
      .onTrue(new InstantCommand(() -> { mIntakeSubsystem.setRollerMotorSpeed(0.25); } ));

    Trigger IntakeRollerStopEjectTrigger 
      = new Trigger(() -> mControllerTertiary.getLeftBumperReleased());
    IntakeRollerStopEjectTrigger
      .onTrue(new InstantCommand(() -> { mIntakeSubsystem.setRollerMotorSpeed(0.0); } ));

    //INTAKE ARM (Manual)

    // Trigger IntakeArmDeployTrigger 
    //   = new Trigger(() -> mControllerTertiary.getYButton());
    // IntakeArmDeployTrigger
    //   .onTrue(new InstantCommand(() -> { mIntakeSubsystem.DeployIntake(); } ));

    // Trigger IntakeArmRetractTrigger 
    //   = new Trigger(() -> mControllerTertiary.getAButton());
    // IntakeArmRetractTrigger
    //   .onTrue(new InstantCommand(() -> { mIntakeSubsystem.RetractIntake(); } ));

    // SHOOTER ARM (Manual)

    // Trigger ShooterArmHoldToMoveUpwardTrigger = new Trigger(()-> mControllerTertiary.getYButton())
    //   .onTrue(new InstantCommand(() -> {
    //     mShooterSubsystem.setSpeed
    //   })).onFalse(new InstantCommand(() -> {

    //   }));

    // Repurposing the A and Y buttons for auto AMP score sequence 
    Trigger scoreTrigger = new Trigger(() -> mControllerTertiary.getRightTriggerHeld());
    scoreTrigger.onTrue(new InstantCommand(() -> {mShooterSubsystem.setFireIntoAmp(true);}));
    scoreTrigger.onFalse(new InstantCommand(() -> {mShooterSubsystem.setFireIntoAmp(false);}));

    Trigger AmpScoreSequenceTrigger = new Trigger(() -> mControllerTertiary.getAButton());
    AmpScoreSequenceTrigger.whileTrue(new ManualAmpScoreCommand(mIntakeSubsystem, mShooterSubsystem));

    //INTAKE ARM & ROLLERS (Semi-Auto)
    Trigger IntakeDeployThenAutoIngestThenRetractTrigger 
      = new Trigger(() -> mControllerTertiary.getBButton());
    IntakeDeployThenAutoIngestThenRetractTrigger
      .whileTrue(sequenceDeployIngestRetract);
      //.whileFalse(sequenceStopRollersAndRetract);

    Trigger IntakeStopRollersAndRetractTrigger 
      = new Trigger(() -> mControllerTertiary.getXButton());
    IntakeStopRollersAndRetractTrigger
      .onFalse(sequenceStopRollersAndRetract);

    Trigger IntakeArmToFireTrigger 
      = new Trigger(() -> Math.abs(mControllerTertiary.getLeftTriggerAxis()) > 0.5);
    IntakeArmToFireTrigger
      .onTrue(sequenceArmToFire)
      .onFalse(sequenceDisarm);

    Trigger IntakeFireNoteTrigger 
      = new Trigger(() -> (Math.abs(mControllerTertiary.getRightTriggerAxis()) > 0.5) && mArmedToFire);
    IntakeFireNoteTrigger.onTrue(sequenceFireNote);
    
    //togggle climber and shoooter ( default shooter; default false )
    // We are currently not using a multi-mode setup. These buttons are FREE.
    Trigger toggleToShooter = new Trigger(() -> {
      return mControllerTertiary.getStartButton();
      //Start maps Shooter
    });
    // toggleToShooter.onTrue(new InstantCommand(() -> {
    //   setForShooterNotClimber(true);
    // }));

    Trigger toggleToClimber = new Trigger(() -> {
      return mControllerTertiary.getBackButton();
      //Back maps climber
    });
    // toggleToClimber.onTrue(new InstantCommand(() -> {
    //   setForShooterNotClimber(false);
    // }));
    
    //DPAD TRIGGERS
    Trigger upDPAD = new Trigger(() -> {
      return mControllerTertiary.getDPadUp();
    });

    Trigger downDPAD = new Trigger(() -> {
      return mControllerTertiary.getDPadDown();
    });

    Trigger leftDPAD = new Trigger(() -> {
      return mControllerTertiary.getDPadLeft();
    });

    Trigger rightDPAD = new Trigger(() -> {
      return mControllerTertiary.getDPadRight();
    });
    
    //checks if DPAD is in shooter(false) or climber(true) mode
    // default is shooter
    // if(!mIsMappedForShooterNotClimber){

    // CLIMBER manual left joystick and right joystick y
    Trigger ClimberTriggerLeft = new Trigger(() -> Math.abs(mControllerTertiary.getLeftDeadbandY()) > 0.15);
    ClimberTriggerLeft.onTrue(new InstantCommand(() -> {
      mClimberSubsystem.setClimberMotorSpeed(mControllerTertiary.getLeftDeadbandY(), climberMotorToSpinEnum.left);
    })).onFalse(new InstantCommand(() -> {
      mClimberSubsystem.setClimberMotorSpeed(0, climberMotorToSpinEnum.left);
    }));
    
    Trigger ClimberTriggerRight = new Trigger(() -> Math.abs(mControllerTertiary.getRightDeadbandY()) > 0.15);
    ClimberTriggerRight.onTrue(new InstantCommand(() -> {
      mClimberSubsystem.setClimberMotorSpeed(mControllerTertiary.getRightDeadbandY(), climberMotorToSpinEnum.right);
    })).onFalse(new InstantCommand(() -> {
      mClimberSubsystem.setClimberMotorSpeed(0, climberMotorToSpinEnum.right);
    }));

    // TRAP climb
    upDPAD.onTrue(new InstantCommand(() -> {
      mShooterSubsystem.setDesiredShooterAngleState(ShooterAngleState.eTrap);
    }));

    downDPAD.onTrue(new InstantCommand(() -> {
      mShooterSubsystem.setDesiredShooterAngleState(ShooterAngleState.eSpeaker);
    }));

    // INTAKE manual
    leftDPAD.onTrue(new InstantCommand(() -> {
      mIntakeSubsystem.RetractIntake();
      // intake in no spin
    }));
    
    rightDPAD.onTrue(new InstantCommand(() -> {
      mIntakeSubsystem.DeployIntake();
      // intake out no spin
    }));

    // when climbing on the chain, the arm is lifted up at a high angle to balance the bot using the wall.
    // to try and shoot into the trap, we must fire backwards while holding the note in the shooter arm.
    Trigger shootBackwards = new Trigger(() -> mControllerTertiary.getYButton());
    shootBackwards.onTrue(new InstantCommand(() -> {
      mShooterSubsystem.setShooterMotorSpeed(-1, -1); // yolo
    })).onFalse(new InstantCommand(() -> {
      mShooterSubsystem.setShooterMotorSpeed(0, 0);
    }));

    // downDPAD.onTrue(new InstantCommand(() -> {
    //   mClimberSubsystem.setClimberMotorSpeed(kClimberMotorSpeed, "both");
    // })).onFalse(new InstantCommand(() -> {
    //   mClimberSubsystem.setClimberMotorSpeed(0, "both");
    // }));

    // leftDPAD.onTrue(new InstantCommand(() -> {
    //   double speed = kClimberMotorSpeed;
    //   if (mControllerTertiary.getRightStickButtonPressed()) {
    //     speed *= -1;
    //   }
    //   mClimberSubsystem.setClimberMotorSpeed(speed, "left");
    // })).onFalse(new InstantCommand(() -> {
    //   mClimberSubsystem.setClimberMotorSpeed(0, "left");
    // }));

    // rightDPAD.onTrue(new InstantCommand(() -> {
    //   double speed = kClimberMotorSpeed;
    //   if (mControllerTertiary.getRightStickButtonPressed()) {
    //     speed *= -1;
    //   }
    //   mClimberSubsystem.setClimberMotorSpeed(speed, "right");
    // })).onFalse(new InstantCommand(() -> {
    //   mClimberSubsystem.setClimberMotorSpeed(0, "right");
    // })); 

    // }
    // else{
      // DPAD Up (in correct mode) -> start to aim at amp

      // Trigger AngleShootMotorPIDTrigger = new Trigger(()->{
      //   return mControllerTertiary.getDPadUp();
      // });
      // AngleShootMotorPIDTrigger.whileTrue(new InstantCommand(()->{
      //   mShooterSubsystem.setDesiredShooterAngleState(ShooterAngleState.eAmp);
      //   // mShooterSubsystem.runShooterAngleSetpointChaser();
      // }).repeatedly()).whileFalse(new InstantCommand(()->{
      //   mShooterSubsystem.setDesiredShooterAngleState(ShooterAngleState.eSpeaker);
      //   // mShooterSubsystem.runShooterAngleSetpointChaser();
      // }).repeatedly());
  
      // // DPAD Down to activate shooter motors.
      // Trigger ShooterMotorTrigger = new Trigger(() -> mControllerTertiary.getDPadDown());
      // ShooterMotorTrigger
      //   .onTrue(new InstantCommand(() -> {
      //       mShooterSubsystem.setShooterMotorSpeed(kShootMotorShootingVelocity, kShootMotorShootingVelocity);
      //     })
      //   ).onFalse(new InstantCommand(()-> {
      //     mShooterSubsystem.setShooterMotorSpeed(0, 0);
      // }));
    }


  // }

  private Command sequenceScoreSpeakerOnce = new SequentialCommandGroup(
    (new DelayCommand(0.1)).andThen
    (new ShooterStageToNoteLoadAngleCommand(mShooterSubsystem)).andThen
    (new DelayCommand(0.2)).andThen
    (new RetractIntakeCommand(mIntakeSubsystem)).andThen
    (new DelayCommand(0.2)).andThen
    (new ShooterMotorsToSpeakerSpeedCommand(mShooterSubsystem)).andThen
    (new DelayCommand(0.2)).andThen
    (new ShooterStageToSpeakerAngleCommand(mShooterSubsystem)).andThen
    (new DelayCommand(0.2)).andThen
    (new InstantCommand(() -> { mArmedToFire = true; })).andThen
    (new DelayCommand(0.1)).andThen
    (new EjectNoteCommand(mIntakeSubsystem)).andThen
    (new DelayCommand(0.1)).andThen
    (new ShooterMotorsOffCommand(mShooterSubsystem)).andThen
    (new DelayCommand(0.2)).andThen
    (new InstantCommand(() -> { mArmedToFire = false; }))

    //  .andThen((sequenceDeployIngestRetract).andThen(new DelayCommand(0.35)).alongWith
    //  (new LinearDriveCommand(mDriveSubsystem, -1.4, 0 ,0))).andThen
   
    // (new LinearDriveCommand(mDriveSubsystem, 1.4,0, 0)).andThen

    // (new DelayCommand(0.1)).andThen
    // (new ShooterStageToNoteLoadAngleCommand(mShooterSubsystem)).andThen
    // (new DelayCommand(0.2)).andThen
    // (new RetractIntakeCommand(mIntakeSubsystem)).andThen
    // (new DelayCommand(0.2)).andThen
    // (new ShooterMotorsToSpeakerSpeedCommand(mShooterSubsystem)).andThen
    // (new DelayCommand(0.2)).andThen
    // (new ShooterStageToSpeakerAngleCommand(mShooterSubsystem)).andThen
    // (new DelayCommand(0.2)).andThen
    // (new InstantCommand(() -> { mArmedToFire = true; })).andThen
    // (new DelayCommand(0.1)).andThen
    // (new EjectNoteCommand(mIntakeSubsystem)).andThen
    // (new DelayCommand(0.1)).andThen
    // (new ShooterMotorsOffCommand(mShooterSubsystem)).andThen
    // (new DelayCommand(0.2)).andThen
    // (new InstantCommand(() -> { mArmedToFire = false; })
    );
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
    

    return sequenceScoreSpeakerOnce;
    // return new AutoAmpScoreSequence(mShooterSubsystem, mIntakeSubsystem);

    // return new DelayCommand(1.0).andThen
    // (new LinearDriveCommand(mDriveSubsystem, 1.0, 1.0, 360.0)).andThen
    //   ((new LinearDriveCommand(mDriveSubsystem, -1.0, 1.0, 360.0)).alongWith
    //   (new RetractIntakeCommand(mIntakeSubsystem))).andThen
    // (new LinearDriveCommand(mDriveSubsystem, -1.0, -1.0, 360.0)).andThen
    // (new LinearDriveCommand(mDriveSubsystem, 1.0, -1.0, 360.0));

  //  return new  DelayCommand(1.0).andThen
  //   (new LinearDriveCommand(mDriveSubsystem, 1.0, 1.0, 0.0)).andThen
  //   (new LinearDriveCommand(mDriveSubsystem, 1.0, -2.0, 90.0)).andThen
  //   (new LinearDriveCommand(mDriveSubsystem, 1.0, 2.0, -90.0)).andThen
  //   (new LinearDriveCommand(mDriveSubsystem, 1.0, -1.0, 180)).andThen
  //   (new DelayCommand(2.0)).andThen
  //   (new LinearDriveCommand(mDriveSubsystem, 0.0, 0.0, 45.0)).andThen
  //   (new DelayCommand(2.0)).andThen
  //   (new LinearDriveCommand(mDriveSubsystem, 0.0, 0.0, 90.0)).andThen
  //   (new DelayCommand(2.0)).andThen
  //   (new LinearDriveCommand(mDriveSubsystem, 0.0, 0.0, 90.0)).andThen
  //   (new DelayCommand(2.0)).andThen
  //   (new LinearDriveCommand(mDriveSubsystem, 1.0, 0.0, 45.0)); 

    // return new  DelayCommand(1.0).andThen
    // (new LinearDriveCommand(mDriveSubsystem, 1.0, 1.0, 0.0)).andThen
    // (new LinearDriveCommand(mDriveSubsystem, 1.0, -2.0, 90.0)).andThen
    // (new LinearDriveCommand(mDriveSubsystem, 1.0, 2.0, -90.0)).andThen
    // (new LinearDriveCommand(mDriveSubsystem, 1.0, -1.0, 180)).andThen
    // (new LinearDriveCommand(mDriveSubsystem, 1.0, 0.0, 45.0)).andThen
    // (new LinearDriveCommand(mDriveSubsystem, 1.0, 0.0, 45.0)); TEST



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

  public static void triggerScoring(boolean trigger) {
    isScoreTriggered = trigger;
  }

  public static boolean isScoreTriggered() {
    return isScoreTriggered;
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

  public void calibrateIntakeArm() {
    mIntakeSubsystem.calibrateArm();
  }

  /**
   * Reconfigures button bindings when start button and back button is pressed
   * This causes a core dump fatal error. Commenting out for now
   * @param toggle
   */
  // public void setForShooterNotClimber(boolean toggle){
  //   mIsMappedForShooterNotClimber = toggle;
  //   configureButtonBindings();
  // }

}

