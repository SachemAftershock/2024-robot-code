package frc.robot.commands.ArmToShoot;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.ChangeDesiredIntakeStateCommandGroup;
import frc.robot.commands.Intake.IntakeRollerCommand;
import frc.robot.enums.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;

public class ArmAmpCommandGroup extends SequentialCommandGroup {
    private RobotContainer mRobotContainer = RobotContainer.getInstance();
    private IntakeSubsystem mIntakeSubsystem;
    private double outRollerVelocity = 5;
    public ArmAmpCommandGroup(IntakeSubsystem mIntakeSubsystem) {
        // Add commands to be run sequentially
        this.mIntakeSubsystem = mIntakeSubsystem;
        addCommands(
            new ChangeDesiredIntakeStateCommandGroup(mIntakeSubsystem, IntakeState.eDeployed),
            new IntakeRollerCommand(outRollerVelocity, mIntakeSubsystem)
        );
    }
}