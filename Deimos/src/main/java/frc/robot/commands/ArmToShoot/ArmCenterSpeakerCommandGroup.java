package frc.robot.commands.ArmToShoot;
import static frc.robot.Constants.ShooterConstants.kArmAmpRollerVelocity;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.ChangeDesiredIntakeStateCommandGroup;
import frc.robot.commands.Intake.IntakeRollerCommand;
import frc.robot.enums.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;

public class ArmCenterSpeakerCommandGroup extends SequentialCommandGroup {
    private RobotContainer mRobotContainer = RobotContainer.getInstance();
    private IntakeSubsystem mIntakeSubsystem;
    private double mArmAmpRollerVelocity = kArmAmpRollerVelocity;
    public ArmCenterSpeakerCommandGroup(IntakeSubsystem mIntakeSubsystem) {
        // Add commands to be run sequentially
        this.mIntakeSubsystem = mIntakeSubsystem;
        addCommands(
            new ChangeDesiredIntakeStateCommandGroup(mIntakeSubsystem, IntakeState.eOut),
            new IntakeRollerCommand(mArmAmpRollerVelocity, mIntakeSubsystem)
        );
    }
}