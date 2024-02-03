// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ShooterConstants.kLeftShootMotorSpeed;
import static frc.robot.Constants.DriveConstants.kXboxJoystickDeadband;
import static frc.robot.Constants.DriveConstants.kXboxTriggerDeadband;
import static frc.robot.Constants.ShooterConstants.*;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.lib.AftershockXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants.*;
import frc.robot.Constants.ShooterConstants.*;
import frc.robot.Constants.DriveConstants.CardinalDirection;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.Drive.FollowTrajectoryCommandFactory;
import frc.robot.commands.Drive.LinearDriveCommand;
import frc.robot.commands.Drive.ManualDriveCommand;
import frc.robot.commands.Drive.RotateDriveCommand;
import frc.robot.commands.SetManualControlModeCommand;
import frc.robot.commands.ZeroRobotCommandGroup;
import frc.robot.commands.Intake.IntakePIDCommand;
import frc.robot.commands.Shooter.ManualShooterAngleCommand;
import frc.robot.commands.Shooter.ShooterAngleCommandGroup;
import frc.robot.commands.Shooter.ShooterPIDCommand;
import frc.robot.commands.Shooter.ShooterRollerCommand;
import frc.robot.enums.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();
  private IntakeSubsystem mIntakeSubsystem = IntakeSubsystem.getInstance();
  private ShooterSubsystem mShooterSubsystem = ShooterSubsystem.getInstance();
  private ClimberSubsystem mClimberSubsystem = ClimberSubsystem.getInstance();

  // Driver 1 - drives
  private final CommandJoystick mControllerPrimary = new CommandJoystick(0);
  private final CommandJoystick mControllerSecondary = new CommandJoystick(1);
  // Driver 2 - manages subsystems
  private final AftershockXboxController mControllerTertiary = new AftershockXboxController(2);

  /*
  Shuffleboard configure bindings

  For shuffleboard, to make a key show a value on some tab we:
    1. make a tab with getTab
    2. add some key/value pair with .add (or .addBoolean etc. for a supplier), and call getEntry on that (returns GenericEntry)
    3. update GenericEntry when you need to.

    Example: read second example in https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/layouts-with-code/sending-data.html
  */
    private ShuffleboardTab mErrorTab = Shuffleboard.getTab("Error");
    private GenericEntry bindingsExist = mErrorTab.add("Current state is bound correctly", true).getEntry();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    mDriveSubsystem.setDefaultCommand(new ManualDriveCommand(
        mDriveSubsystem,
        () -> -modifyAxis(mControllerPrimary.getY()) * DriveConstants.kMaxVelocityMetersPerSecond * 0.7,
        () -> -modifyAxis(mControllerPrimary.getX()) * DriveConstants.kMaxVelocityMetersPerSecond * 0.7,
        () -> -modifyAxis(mControllerSecondary.getTwist()) * DriveConstants.kMaxAngularVelocityRadiansPerSecond * 0.3));
    mIntakeSubsystem.setDefaultCommand(new IntakePIDCommand(mIntakeSubsystem)); //TODO: move to IntakeSubsystem
    mShooterSubsystem.setDefaultCommand(new ShooterPIDCommand(mShooterSubsystem));

  }

  public void initialize() {
    mDriveSubsystem.initialize();
    mIntakeSubsystem.initialize();
    mShooterSubsystem.initialize();
    mClimberSubsystem.initialize();
    setClimberState(ClimberState.eDown);
    setDesiredClimberState(ClimberState.eDown);
    setIntakeState(IntakeState.eRetracted);
    setDesiredIntakeState(IntakeState.eRetracted);
    setShooterState(ShooterState.eSpeaker);
    setDesiredShooterState(ShooterState.eSpeaker);
    setControlState(ControlState.eManualControl);
    setSuperState(null);
  }

  private double rumbleValue = .5;

  // rumbles controller
  public void Rumble(int times) {
    for (int i = 0; i < times; i++) {
      mControllerPrimary.setRumble(null, rumbleValue);
    }
  }

  // Below shown is mutator and accessor methods for robot states and enums, all
  // globally accessible through this robotcontainerclass
  private ControlState mCurrentControlState;

  public void setControlState(ControlState mControlState) {
    this.mCurrentControlState = mControlState;
  }

  public ControlState getControlState() {
    return mCurrentControlState;
  }

  private ClimberState mCurrentClimberState;

  public void setClimberState(ClimberState mCurrentClimberState) {
    this.mCurrentClimberState = mCurrentClimberState;
  }

  public ClimberState getClimberState() {
    return mCurrentClimberState;
  }

  private ClimberState mDesiredClimberState;

  public ClimberState getDesiredClimberState() {
    return mDesiredClimberState;
  }

  public void setDesiredClimberState(ClimberState mDesiredClimberState) {
    this.mDesiredClimberState = mDesiredClimberState;
  }

  private SuperState mCurrentSuperState;

  public void setSuperState(SuperState mCurrentSuperState) {
    this.mCurrentSuperState = mCurrentSuperState;
  }

  public SuperState getSuperState() {
    return mCurrentSuperState;
  }

  private IntakeState mCurrentIntakeState;

  public void setIntakeState(IntakeState mCurrentIntakeState) {
    this.mCurrentIntakeState = mCurrentIntakeState;
  }

  public IntakeState getIntakeState() {
    return mCurrentIntakeState;
  }

  private IntakeState mDesiredIntakeState;

  public void setDesiredIntakeState(IntakeState mDesiredIntakeState) {
    this.mDesiredIntakeState = mDesiredIntakeState;
  }

  public IntakeState getDesiredIntakeState() {
    return mDesiredIntakeState;
  }

  private ShooterState mCurrentShooterState;

  public void setShooterState(ShooterState mCurrentShooterState) {
    this.mCurrentShooterState = mCurrentShooterState;
  }

  public ShooterState getShooterState() {
    return mCurrentShooterState;
  }

  private ShooterState mDesiredShooterState;

  public void setDesiredShooterState(ShooterState mDesiredShooterState) {
    this.mDesiredShooterState = mDesiredShooterState;
  }

  public ShooterState getDesiredShooterState() {
    return mDesiredShooterState;
  }

  public AftershockXboxController getControllerTertiary() {
    return mControllerTertiary;
  }

  private double shooterJogSpeed = 0.2;
  private double intakeJogSpeed = 0.2;

  public void configureButtonBindings() {
    // TODO add button bindings

    mControllerPrimary.button1.onTrue(new SetManualControlModeCommand(true));
    mControllerPrimary.button2.onTrue(new SetManualControlModeCommand(false));

    // When in "automatic control", commands which involve PID movement of
    // mechanisms are availible
    if (getControlState().equals(ControlState.eSemiAutoControl)) {
      mControllerPrimary.button3.onTrue(new ZeroRobotCommandGroup(mShooterSubsystem, mIntakeSubsystem));
      mControllerPrimary.button4
          .onTrue(new ShooterAngleCommandGroup(mShooterSubsystem, /* mIntakeSubsystem, */ ShooterState.eSpeaker));

      mControllerPrimary.button5.onTrue(new ShooterAngleCommandGroup(mShooterSubsystem, ShooterState.eAmp));
      mControllerPrimary.button6.onTrue(new ShooterAngleCommandGroup(mShooterSubsystem, ShooterState.eSafeZone));

    } else if (getControlState().equals(ControlState.eManualControl)) {
      // when in "manual control", commands which involve direct driver control of
      // mechanisms are used
      // Trigger ShooterJogDownTriggerPress = new Trigger(() -> mControllerTertiary.getAButtonPressed());
      // Trigger ShooterJogDownTriggerRelease = new Trigger(() -> mControllerTertiary.getAButtonReleased());

      // ShooterJogDownTriggerPress.onTrue(new ManualShooterAngleCommand(mShooterSubsystem, shooterJogSpeed));
      // ShooterJogDownTriggerRelease.onTrue(new ManualShooterAngleCommand(mShooterSubsystem, 0));

      // Trigger ShooterJogUpTriggerPress = new Trigger(() -> mControllerTertiary.getBButtonPressed());
      // Trigger ShooterJogUpTriggerRelease = new Trigger(() -> mControllerTertiary.getBButtonReleased());

      // ShooterJogUpTriggerPress.onTrue(new ManualShooterAngleCommand(mShooterSubsystem, -shooterJogSpeed));
      // ShooterJogUpTriggerRelease.onTrue(new ManualShooterAngleCommand(mShooterSubsystem, 0));

      // Trigger ShooterWheelsTriggerPress = new Trigger(() -> mControllerTertiary.getXButtonPressed());
      // Trigger ShooterWheelsTriggerRelease = new Trigger(() -> mControllerTertiary.getXButtonReleased());

      // Trigger EjectNote = new Trigger(()-> mControllerTertiary.getXButtonPressed()); //TODO: CHANGE FROM X
      // EjectNote.onTrue(new InstantCommand(mIntakeSubsystem.ejectNote().andThen(mIntakeSubsystem.setRollerMotorSpeed(0.0))));

      // Trigger IngestNote = new Trigger(()-> mControllerTertiary.getXButtonPressed()); //TODO: CHANGE FROM X
      // EjectNote.onTrue(new InstantCommand(mIntakeSubsystem.ingestNote().andThen(mIntakeSubsystem.setRollerMotorSpeed(0.0))));

      // ShooterJogUpTriggerPress
      //     .onTrue(new ShooterRollerCommand(kLeftShootMotorSpeed, kRightShootMotorSpeed, mShooterSubsystem));// TODO: add  constants
      // ShooterJogUpTriggerRelease.onTrue(new ManualShooterAngleCommand(mShooterSubsystem, 0));
     
      //  Trigger ShooterLeftWheelsTriggerPress = new Trigger(() -> mControllerTertiary.getLeftTriggerPressed());
      //  Trigger ShooterLeftWheelsTriggerRelease = new Trigger(() -> mControllerTertiary.getLeftTriggerReleased());

      //  Trigger ShooterRightWheelsTriggerPress = new Trigger(() -> mControllerTertiary.getRightTriggerPressed());
      //  Trigger ShooterRightWheelsTriggerRelease = new Trigger(() -> mControllerTertiary.getRightTriggerReleased());

      // ShooterLeftWheelsTriggerPress.onTrue(new InstantCommand(() -> {
      //   if(Math.abs(mControllerTertiary.getLeftTriggerAxis()) >= 0.05){
      //     mShooterSubsystem.spinShooterMotors(mControllerTertiary.getLeftTriggerAxis(), 0);
      //   }
      // }));
      if (mControllerTertiary.getLeftTriggerAxis() >= kXboxTriggerDeadband && mControllerTertiary.getRightTriggerAxis() >= kXboxTriggerDeadband){
        mShooterSubsystem.spinShooterMotors(mControllerTertiary.getLeftTriggerAxis(), mControllerTertiary.getRightTriggerAxis());
      } else if (mControllerTertiary.getLeftTriggerAxis() >= kXboxTriggerDeadband){
        mShooterSubsystem.spinShooterMotors(mControllerTertiary.getLeftTriggerAxis(), 0);
      } else if (mControllerTertiary.getRightTriggerAxis() >= kXboxTriggerDeadband){
        mShooterSubsystem.spinShooterMotors(0, mControllerTertiary.getRightTriggerAxis()); 
      } else {
        mShooterSubsystem.spinShooterMotors(0, 0);
      }
      if( Math.abs(mControllerTertiary.getLeftY()) <= kXboxJoystickDeadband){
        mShooterSubsystem.setShooterArmMotorSpeed(-mControllerTertiary.getLeftY());
      }

      // ShooterLeftWheelsTriggerRelease.onTrue(new InstantCommand(() -> {
      //   mShooterSubsystem.spinShooterMotors(0, 0);
      // }));

      // ShooterRightWheelsTriggerPress.onTrue(new InstantCommand(() -> {
      //   if(Math.abs(mControllerTertiary.getRightTriggerAxis()) >= 0.05){
          
      //   }
      //   mShooterSubsystem.spinShooterMotors(0, mControllerTertiary.getRightTriggerAxis());
      // }));

      // ShooterRightWheelsTriggerRelease.onTrue(new InstantCommand(() -> {
      //   mShooterSubsystem.spinShooterMotors(0, 0);
      // }));

      /*
        Intake
      */
      // FIX ME move intake arm with tertiarycontroller right stick

      Trigger IntakeRollerIngestTriggerPress = new Trigger(() -> mControllerTertiary.getLeftBumper());
      Trigger IntakeRollerIngestTriggerRelease = new Trigger(() -> mControllerTertiary.getLeftBumper());

      Trigger IntakeRollerEjectTriggerPress = new Trigger(() -> mControllerTertiary.getRightBumper());
      Trigger IntakeRollerEjectTriggerRelease = new Trigger(() -> mControllerTertiary.getRightBumper());

      IntakeRollerIngestTriggerPress.onTrue(new InstantCommand(() -> { 
        mIntakeSubsystem.setRollerMotorSpeed(0.4);
      }));

      IntakeRollerIngestTriggerRelease.onTrue(new InstantCommand(() -> { 
        mIntakeSubsystem.setRollerMotorSpeed(0); 
      }));

      IntakeRollerEjectTriggerPress.onTrue(new InstantCommand(() -> { 
        mIntakeSubsystem.setRollerMotorSpeed(-0.4);
      }));

      IntakeRollerEjectTriggerRelease.onTrue(new InstantCommand(() -> { 
        mIntakeSubsystem.setRollerMotorSpeed(0); 
      }));

    } else {
      // TODO: Add else statement that makes an error signal to the dashboard
      bindingsExist.setBoolean(false);
    }
  }

  public Command getAutonomousCommand() {
    TrajectoryConfig config = new TrajectoryConfig(
        DriveConstants.kMaxVelocityMetersPerSecond * 0.3,
        DriveConstants.kMaxAccelerationMetersPerSecondSquared);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(),
        List.of(
            new Translation2d(2.0, -0.5)
        // new Translation2d(0, 1.2)//,
        // new Translation2d(1, 1.5)
        // new Translation2d(2.2,0)
        ),
        new Pose2d(1.0, -0.4, new Rotation2d()),
        config);
    Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(),
        List.of(
            new Translation2d(0.5, 1.5),
            new Translation2d(1.0, 1.5),
            new Translation2d(1.5, 1.5)),
        new Pose2d(1.5, 1.5, new Rotation2d()),
        config);
    // return new RotateDriveCommand(mDriveSubsystem, 90);
    // mDriveSubsystem.zeroGyroscope();
    return new DelayCommand(0.15).andThen(new LinearDriveCommand(mDriveSubsystem, 4.0, CardinalDirection.eX))
        .andThen(new DelayCommand(.2)).andThen(new LinearDriveCommand(mDriveSubsystem, -4.0, CardinalDirection.eX)); // was
                                                                                                                     // 2.0
    /**
     * return new SequentialCommandGroup(
     * new LinearDriveCommand(mDriveSubsystem, 2, CardinalDirection.eX),
     * new RotateDriveCommand(mDriveSubsystem, 180),
     * new LinearDriveCommand(mDriveSubsystem, 2, CardinalDirection.eX)
     * //FollowTrajectoryCommandFactory.generateCommand(mDriveSubsystem,
     * trajectory),
     * //new RotateDriveCommand(mDriveSubsystem, 90),
     * //FollowTrajectoryCommandFactory.generateCommand(mDriveSubsystem,
     * trajectory2),
     * //new RotateDriveCommand(mDriveSubsystem, -30)
     * );
     **/
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
    if (DriveConstants.kSquareAxis) {
      value = Math.copySign(value * value, value);
    }
    return value;
  }

  // below is the instance variable for commands and subsystems to access
  // robotcontainer methods
  public static RobotContainer mInstance;

  public static RobotContainer getInstance() {
    if (mInstance == null)
      mInstance = new RobotContainer();
    return mInstance;
  }
}
