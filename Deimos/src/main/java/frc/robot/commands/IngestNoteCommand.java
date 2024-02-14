package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IngestNoteCommand extends Command {
    private IntakeSubsystem mIntake;

    public IngestNoteCommand(IntakeSubsystem intake){
        mIntake = intake;
        addRequirements(mIntake);
    }

    public void initialize() {
        System.out.println("IngestNoteCommand INIT");
    }

    public void execute() {
        System.out.println("_______INTAKE_______");
        System.out.println("IngestNoteCommand EXE");  
        mIntake.ingestNote();  
    }

    public boolean isFinished() {
        System.out.println("IngestNoteCommand FIN");
        return mIntake.isNoteCaptive();
    }

    public void end(){
        System.out.println("IngestNoteCommand END");
    }

}
