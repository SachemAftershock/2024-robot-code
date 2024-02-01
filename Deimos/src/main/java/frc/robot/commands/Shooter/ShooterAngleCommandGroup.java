package frc.robot.commands.Shooter;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.ChangeDesiredIntakeState;
import frc.robot.commands.Intake.IntakePIDCommand;
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
    public ShooterAngleCommandGroup(ShooterSubsystem mShooterSubsystem, ShooterState desiredShooterState) {
        // Add commands to be run sequentially
        this.mShooterSubsystem = mShooterSubsystem;
        addCommands(
            new ChangeDesiredShooterState(desiredShooterState, mShooterSubsystem),
            new ShooterPIDCommand(mShooterSubsystem)
        );
    }
}