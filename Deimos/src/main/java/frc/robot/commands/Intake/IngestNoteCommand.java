package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * IntakeNoteCommand
 */
public class IngestNoteCommand extends Command {
    private IntakeSubsystem mIntakeSubsystem;

    public IngestNoteCommand(IntakeSubsystem intakeSubsystem) {
        mIntakeSubsystem = intakeSubsystem;
        addRequirements(mIntakeSubsystem);
    }

    @Override
    public void execute() {
        mIntakeSubsystem.ingestNote();
    }
}