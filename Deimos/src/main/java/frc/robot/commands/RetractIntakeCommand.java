package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RetractIntakeCommand extends Command {
    private IntakeSubsystem mIntake;

    public RetractIntakeCommand(IntakeSubsystem intake){
        mIntake = intake;
        addRequirements(mIntake);
    }
    public void initialize() {
        System.out.println("INIT");

    }
    public void execute() {
        System.out.println("_______INTAKE_______");
        System.out.println("EXE");

    }
    public boolean isFinished() {
        System.out.println("FIN");
        return true;
    }
    public void end(){
        System.out.println("END");
    }

}
