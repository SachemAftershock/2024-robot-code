package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class SafeZoneIntakeCommand extends Command {

    final boolean showPrints = true;		

    private IntakeSubsystem mIntake;

    public SafeZoneIntakeCommand(IntakeSubsystem intake){
        mIntake = intake;
        addRequirements(mIntake);
    }

    public void initialize() {
        if (showPrints) System.out.println("SafeZoneIntakeCommand INIT");
        mIntake.RetractIntake();
    }

    public void execute() {
        if (showPrints) System.out.println("_______INTAKE_______");
        if (showPrints) System.out.println("SafeZoneIntakeCommand EXE");    
    }

    public boolean isFinished() {
        if (showPrints) System.out.println("SafeZoneIntakeCommand FIN");
        return mIntake.getIntakeArmState() == IntakeSubsystem.IntakeArmPositionEnum.eSafeZone;
    }

    public void end(){
        if (showPrints) System.out.println("SafeZoneIntakeCommand END");
    }

}