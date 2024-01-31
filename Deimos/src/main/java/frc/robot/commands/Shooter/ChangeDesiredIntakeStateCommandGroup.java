package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ChangeDesiredIntakeStateCommandGroup extends SequentialCommandGroup {
    private RobotContainer mRobotContainer = RobotContainer.getInstance();
    private ShooterSubsystem mShooterSubsystem;
    private ShooterState mShooterState;
    private double rollerVelocity = 0;
    public ChangeDesiredIntakeStateCommandGroup(IntakeSubsystem mIntakeSubsystem, ShooterState mShooterState) {
        // Add commands to be run sequentially
        this.mShooterSubsystem = mShooterSubsystem;
        this.mShooterState = mShooterState;
        addCommands(
            new ChangeDesiredShooterState(mShooterState, mShooterSubsystem),
            new ShooterPIDCommand(mShooterSubsystem)
        );
    }
}