package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoAmpScoreSequence extends SequentialCommandGroup {


    public AutoAmpScoreSequence(ShooterSubsystem mShooterSubsystem, IntakeSubsystem mIntakeSubsystem) {
        addCommands(
            new AutoAmpScoreCommand(mIntakeSubsystem, mShooterSubsystem),
            new EjectFromShooterCommand(mShooterSubsystem)
        );
    }
    
}
