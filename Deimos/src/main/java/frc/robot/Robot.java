// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightManagerSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private LampController mLampController = LampController.getInstance();
  // private ChoreoManager mChoreoManager = ChoreoManager.getInstance();
  private Recorder mRecorder = Recorder.getInstance();

  private int count = 0;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    CameraServer.startAutomaticCapture();
    m_robotContainer = new RobotContainer();
    m_robotContainer.initialize();
    // mLampController.setPulse(3, 0.75, 0.25, 2.0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    if ((count++ % 50) == 0) {
      LimelightManagerSubsystem.getInstance().outputTelemetry();
      count = 0;
    }

    mLampController.run();
    // mChoreoManager.updatePose();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    mLampController.setPulse(0,0,0,0,true);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    mRecorder.loadFromFile("Center4note", false);
    m_robotContainer.initialize();
    // m_robotContainer.calibrateIntakeArm();
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    // m_autonomousCommand.schedule();
    // }
    Constants.DriverStationConstants.updateAllianceColorAndLocation();
    Recorder.setIsPlaying(true);
    // mLampController.setPulse(3, 0.5, 0.5, 0.5);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    mRecorder.playNextFrame();
  }

  @Override
  public void teleopInit() {
    System.out.println("teleopINIT");
    m_robotContainer.initialize();

    // m_robotContainer.calibrateIntakeArm();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    CommandScheduler.getInstance().cancelAll();
    if (!IntakeSubsystem.IsBothBeamBreakersBeenBroken()) {
      System.out.println("WARNING: Auto ended without both intake beem breakers being broken!!");
    }

    Constants.DriverStationConstants.updateAllianceColorAndLocation();

    // mLampController.setPulse(3, 0.25, 0.75, 0.5);

    // TODO: but a guard here, do not do again after auto mode if this is running on
    // the real field managemeent system.
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void teleopExit() {
    mLampController.setPulse(0, 0, 0, 0, true);
  }

  @Override
  public void testExit() {
    mLampController.setPulse(0, 0, 0, 0, true);
  }

  @Override
  public void autonomousExit() {
    Recorder.setIsPlaying(false);
    mLampController.setPulse(0, 0, 0, 0, true);

  }
}
