package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.ShooterAngleState;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterStageToSpeakerAngleCommand extends Command {

    final boolean showPrints = true;		

    private ShooterSubsystem mShooter;

    public ShooterStageToSpeakerAngleCommand(ShooterSubsystem shooter){
        mShooter = shooter;
        addRequirements(mShooter);
    }

    public void initialize() {
        if (showPrints) System.out.println("ShooterStageToSpeakerAngleCommand INIT");
        mShooter.setDesiredShooterAngleState(ShooterAngleState.eSpeaker);
    }

    public void execute() {
        if (showPrints)  System.out.println("_______SHOOTER_______");
        if (showPrints)  System.out.println("ShooterStageToSpeakerAngleCommand EXE");    
    }

    public boolean isFinished() {
        if (showPrints) System.out.println("ShooterStageToSpeakerAngleCommand FIN");
        return mShooter.getCurrentShooterAngleState() == ShooterAngleState.eSpeaker;
    }

    public void end(){
        if (showPrints) System.out.println("ShooterStageToSpeakerAngleCommand END");
    }

}
