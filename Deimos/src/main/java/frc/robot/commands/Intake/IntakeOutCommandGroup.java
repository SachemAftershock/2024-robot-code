package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.enums.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOutCommandGroup extends ParallelCommandGroup {
    private RobotContainer mRobotContainer = RobotContainer.getInstance();
    private IntakeSubsystem mIntakeSubsystem;
    private double outRollerVelocity = 5;
    public IntakeOutCommandGroup(IntakeSubsystem mIntakeSubsystem) {
        // Add commands to be run sequentially
        this.mIntakeSubsystem = mIntakeSubsystem;
        addCommands(
            new ChangeDesiredIntakeStateCommandGroup(mIntakeSubsystem, IntakeState.eOut),
            new IntakeRollerCommand(outRollerVelocity, mIntakeSubsystem)
        );
    }
}