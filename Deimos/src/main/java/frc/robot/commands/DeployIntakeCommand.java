package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntakeCommand extends Command {

    final boolean showPrints = false;		

    private IntakeSubsystem mIntake;

    public DeployIntakeCommand(IntakeSubsystem intake){
        mIntake = intake;
        addRequirements(mIntake);
    }

    public void initialize() {
        if (showPrints) System.out.println("DeployIntakeCommand INIT");
        mIntake.DeployIntake();
    }

    public void execute() {
        if (showPrints)  System.out.println("_______INTAKE_______");
        if (showPrints)  System.out.println("DeployIntakeCommand EXE");    
    }

    public boolean isFinished() {
        if (showPrints) System.out.println("DeployIntakeCommand FIN");
        return mIntake.getIntakeArmState() == IntakeSubsystem.IntakeArmPositionEnum.eDeployed;
    }

    public void end(){
        if (showPrints) System.out.println("DeployIntakeCommand END");
    }

}
