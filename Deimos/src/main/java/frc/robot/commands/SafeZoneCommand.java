package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.ShooterAngleState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SafeZoneCommand extends Command {

    final boolean showPrints = true;		

    private IntakeSubsystem mIntakeSubsystem;
    private ShooterSubsystem mShooterSubsystem;

    public SafeZoneCommand(IntakeSubsystem intake, ShooterSubsystem shooter){
        mIntakeSubsystem = intake;
        mShooterSubsystem = shooter;
        addRequirements(mIntakeSubsystem/*, mShooterSubsystem*/); // don't want to freeze up shooter.
    }

    public void initialize() {
        if (showPrints) System.out.println("SafeZoneCommand INIT");
        mIntakeSubsystem.RetractIntake();
        mShooterSubsystem.setDesiredShooterAngleState(ShooterAngleState.eSafeZone);
    }

    public void execute() {
        if (showPrints) System.out.println("_______INTAKE_______");
        if (showPrints) System.out.println("SafeZoneCommand EXE");
    }

    public boolean isFinished() {
        if (showPrints) System.out.println("SafeZoneCommand FIN");
        return mIntakeSubsystem.getIntakeArmState() == IntakeSubsystem.IntakeArmPositionEnum.eSafeZone
            && mShooterSubsystem.getCurrentShooterAngleState() == ShooterAngleState.eSafeZone;
    }

    public void end(){
        if (showPrints) System.out.println("SafeZoneCommand END");
    }

}