package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RetractIntakeCommand extends Command {

    final boolean showPrints = false;		

    private IntakeSubsystem mIntake;

    public RetractIntakeCommand(IntakeSubsystem intake){
        mIntake = intake;
        addRequirements(mIntake);
    }

    public void initialize() {
        if (showPrints) System.out.println("RetractIntakeCommand INIT");
        mIntake.RetractIntake();
    }

    public void execute() {
        if (showPrints) System.out.println("_______INTAKE_______");
        if (showPrints) System.out.println("RetractIntakeCommand EXE");    
    }

    public boolean isFinished() {
        if (showPrints) System.out.println("RetractIntakeCommand FIN");
        return mIntake.getIntakeArmState() == IntakeSubsystem.IntakeArmPositionEnum.eRetracted;
    }

    public void end(){
        if (showPrints) System.out.println("RetractIntakeCommand END");
    }

}
