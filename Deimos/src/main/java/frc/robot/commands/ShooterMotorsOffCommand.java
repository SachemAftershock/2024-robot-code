package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterMotorsOffCommand extends Command {

    final boolean showPrints = false;		

    private ShooterSubsystem mShooter;

    public ShooterMotorsOffCommand(ShooterSubsystem shooter){
        mShooter = shooter;
        addRequirements(mShooter);
    }

    public void initialize() {
        if (showPrints) System.out.println("ShooterMotorsOffCommand INIT");
        mShooter.setShooterMotorSpeed(0,0);
    }

    public void execute() {
        if (showPrints)  System.out.println("_______SHOOTER_______");
        if (showPrints)  System.out.println("ShooterMotorsOffCommand EXE");    
    }

    public boolean isFinished() {
        if (showPrints) System.out.println("ShooterMotorsOffCommand FIN");
        return true;
    }

    public void end(){
        if (showPrints) System.out.println("ShooterMotorsOffCommand END");
    }

}
