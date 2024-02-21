package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterMotorsToSpeakerSpeedCommand extends Command {

    final boolean showPrints = true;		

    private ShooterSubsystem mShooter;

    public ShooterMotorsToSpeakerSpeedCommand(ShooterSubsystem shooter){
        mShooter = shooter;
        addRequirements(mShooter);
    }

    public void initialize() {
        if (showPrints) System.out.println("ShooterMotorsToSpeakerSpeedCommand INIT");
        mShooter.setShooterMotorSpeed(kShootMotorShootingVelocity,kShootMotorShootingVelocity);
    }

    public void execute() {
        if (showPrints)  System.out.println("_______SHOOTER_______");
        if (showPrints)  System.out.println("ShooterMotorsToSpeakerSpeedCommand EXE");    
    }

    public boolean isFinished() {
        if (showPrints) System.out.println("ShooterMotorsToSpeakerSpeedCommand FIN");
        return true;
    }

    public void end(){
        if (showPrints) System.out.println("ShooterMotorsToSpeakerSpeedCommand END");
    }

}
