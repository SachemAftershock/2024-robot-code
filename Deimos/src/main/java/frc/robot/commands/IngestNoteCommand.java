package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IngestNoteCommand extends Command {

    final boolean showPrints = false;	

    private IntakeSubsystem mIntake;

    public IngestNoteCommand(IntakeSubsystem intake){	

        mIntake = intake;
        addRequirements(mIntake);
    }

    public void initialize() {
        if (showPrints) System.out.println("IngestNoteCommand INIT");
    }

    public void execute() {
        if (showPrints) System.out.println("_______INTAKE_______");
        if (showPrints) System.out.println("IngestNoteCommand EXE");  
        mIntake.ingestNote();  
    }

    public boolean isFinished() {
        if (showPrints) System.out.println("IngestNoteCommand FIN");
        return mIntake.isNoteCaptive();
    }

    public void end(){
        mIntake.setRollerMotorSpeed(0);
        if (showPrints) System.out.println("IngestNoteCommand END");
    }

}
