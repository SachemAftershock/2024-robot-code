package frc.robot.commands.Shooter;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.ChangeDesiredIntakeState;
import frc.robot.commands.Intake.IntakeRollerCommand;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterAngleCommandGroup extends SequentialCommandGroup {
    private RobotContainer mRobotContainer = RobotContainer.getInstance();
    private ShooterSubsystem mShooterSubsystem;
    private IntakeSubsystem mIntakeSubsystem;
    private double rollerVelocity = 5;
    private IntakeState previousIntakeState;
    public ShooterAngleCommandGroup(ShooterSubsystem mShooterSubsystem, IntakeSubsystem mIntakeSubsystem, ShooterState desiredShooterState) {
        // Add commands to be run sequentially
        this.mShooterSubsystem = mShooterSubsystem;
        this.mIntakeSubsystem = mIntakeSubsystem;
        previousIntakeState = mRobotContainer.getIntakeState();
        addCommands(
            new ChangeDesiredIntakeState(IntakeState.eSafeShooterMovement, mIntakeSubsystem),
            new ChangeDesiredShooterState(desiredShooterState, mShooterSubsystem),
            new ChangeDesiredIntakeState(m, mIntakeSubsystem)//chang eto get current intake state in current shooter enum
        );
    }
}