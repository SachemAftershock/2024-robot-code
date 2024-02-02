package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * IntakeNoteCommand
 */
public class EjectNoteCommand extends Command {
    private IntakeSubsystem mIntakeSubsystem;

    public EjectNoteCommand(IntakeSubsystem intakeSubsystem) {
        mIntakeSubsystem = intakeSubsystem;
        addRequirements(mIntakeSubsystem);
    }
    
    @Override
    public void execute() {
        mIntakeSubsystem.ejectNote();
    }
}