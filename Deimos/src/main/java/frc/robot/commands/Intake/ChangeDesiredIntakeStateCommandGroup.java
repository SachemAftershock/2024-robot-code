package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.enums.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;

public class ChangeDesiredIntakeStateCommandGroup extends SequentialCommandGroup {
    private RobotContainer mRobotContainer = RobotContainer.getInstance();
    private IntakeSubsystem mIntakeSubsystem;
    private IntakeState mDesiredIntakeState;
    private double rollerVelocity = 0;
    public ChangeDesiredIntakeStateCommandGroup(IntakeSubsystem mIntakeSubsystem, IntakeState mDesiredIntakeState) {
        // Add commands to be run sequentially
        this.mIntakeSubsystem = mIntakeSubsystem;
        this.mDesiredIntakeState = mDesiredIntakeState;
        addCommands(
            new ChangeDesiredIntakeState(mIntakeSubsystem, mDesiredIntakeState),
            new IntakePIDCommand(mIntakeSubsystem)
        );
    }
}