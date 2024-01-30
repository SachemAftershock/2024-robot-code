package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.ChangeDesiredIntakeState;
import frc.robot.commands.Intake.IntakeRollerCommand;
import frc.robot.commands.Shooter.ChangeDesiredShooterState;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
//Purpose: to bring the robot to its stow position safely, and reset states to default
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
            new ChangeDesiredIntakeState(IntakeState.eSafeShooterMovement, mIntakeSubsystem),
            new runIntakePID(),
            new InstantCommand(() -> {
                mShooterSubsystem.spinShooterMotors(0, 0);
            }),
            new ChangeDesiredShooterState(ShooterState.eSpeaker, mShooterSubsystem),
            new runShooterPID(),
            new ChangeDesiredIntakeState(mCurrentShooterState., mIntakeSubsystem)//chang eto get current intake state in current shooter enum
        );
    }
}