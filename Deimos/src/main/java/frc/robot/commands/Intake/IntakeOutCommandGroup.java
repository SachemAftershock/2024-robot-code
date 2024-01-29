package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOutCommandGroup extends SequentialCommandGroup {
    private RobotContainer mRobotContainer = RobotContainer.getInstance();
    private IntakeSubsystem mIntakeSubsystem;
    private double rollerVelocity = 5;
    public IntakeOutCommandGroup(IntakeSubsystem mIntakeSubsystem) {
        // Add commands to be run sequentially
        this.mIntakeSubsystem = mIntakeSubsystem;
        addCommands(
            new ChangeDesiredIntakeState(mRobotContainer.getShooterState().getIntakeState(), mIntakeSubsystem),//change to intakestate.eout
            new IntakeRollerCommand(rollerVelocity, mIntakeSubsystem)
        );
    }
}