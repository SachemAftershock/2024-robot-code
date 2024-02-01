package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.ChangeDesiredIntakeStateCommandGroup;
import frc.robot.commands.Intake.IntakePIDCommand;
import frc.robot.commands.Intake.IntakeRollerCommand;
import frc.robot.commands.Shooter.ChangeDesiredShooterState;
import frc.robot.commands.Shooter.ShooterAngleCommandGroup;
import frc.robot.commands.Shooter.ShooterPIDCommand;
import frc.robot.commands.Shooter.ShooterRollerCommand;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
/**
 * Purpose: to bring the robot to its stow position safely, and reset states to default
 */
public class ZeroRobotCommandGroup extends SequentialCommandGroup {
    private RobotContainer mRobotContainer = RobotContainer.getInstance();
    private ShooterSubsystem mShooterSubsystem;
    private IntakeSubsystem mIntakeSubsystem;
    private double rollerVelocity = 0;
    private IntakeState previousIntakeState;
    public ZeroRobotCommandGroup(ShooterSubsystem mShooterSubsystem, IntakeSubsystem mIntakeSubsystem) {
        // Add commands to be run sequentially
        this.mShooterSubsystem = mShooterSubsystem;
        this.mIntakeSubsystem = mIntakeSubsystem;
        previousIntakeState = mRobotContainer.getIntakeState();
        addCommands(
            new InstantCommand(() -> {
                CommandScheduler.getInstance().cancelAll();
            }),
            new ShooterRollerCommand(0,0,mShooterSubsystem),
            new ChangeDesiredIntakeStateCommandGroup(mIntakeSubsystem,IntakeState.eIn),
            new ShooterAngleCommandGroup(mShooterSubsystem, ShooterState.eSpeaker)
        );
    }
}