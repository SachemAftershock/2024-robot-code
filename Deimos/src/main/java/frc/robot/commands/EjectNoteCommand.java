package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class EjectNoteCommand extends Command {
    private IntakeSubsystem mIntake;

    public EjectNoteCommand(IntakeSubsystem intake){
        mIntake = intake;
        addRequirements(mIntake);
    }

    public void initialize() {
        System.out.println("EjectNoteCommand INIT");
    }

    public void execute() {
        System.out.println("_______INTAKE_______");
        System.out.println("EjectNoteCommand EXE");  
        mIntake.ingestNote();  
    }

    public boolean isFinished() {
        System.out.println("EjectNoteCommand FIN");
        return mIntake.isIntakeEmpty();
    }

    public void end(){
        System.out.println("EjectNoteCommand END");
    }

}
