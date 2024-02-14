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
        System.out.println("RetractIntakeCommand INIT");
        mIntake.RetractIntake();
    }

    public void execute() {
        System.out.println("_______INTAKE_______");
        System.out.println("RetractIntakeCommand EXE");    
    }

    public boolean isFinished() {
        System.out.println("RetractIntakeCommand FIN");
        return mIntake.getIntakeArmState() == IntakeSubsystem.IntakeArmPositionEnum.eRetracted;
    }

    public void end(){
        System.out.println("RetractIntakeCommand END");
    }

}
