package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.enums.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeInCommandGroup extends SequentialCommandGroup {
    private RobotContainer mRobotContainer = RobotContainer.getInstance();
    private IntakeSubsystem mIntakeSubsystem;

    private double rollerVelocity = 0;

    public IntakeInCommandGroup(IntakeSubsystem mIntakeSubsystem) {
        // Add commands to be run sequentially
        this.mIntakeSubsystem = mIntakeSubsystem;
        addCommands(
            new ChangeDesiredIntakeStateCommandGroup(mIntakeSubsystem, IntakeState.eIn),
            new IntakeRollerCommand(rollerVelocity, mIntakeSubsystem)
        );
    }
}