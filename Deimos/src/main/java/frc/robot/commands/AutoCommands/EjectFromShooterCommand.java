package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.ShooterAngleState;
import frc.robot.subsystems.ShooterSubsystem;

public class EjectFromShooterCommand extends Command {

    private final ShooterSubsystem mShooterSubsystem;


    public EjectFromShooterCommand(ShooterSubsystem shooter) {
        mShooterSubsystem = shooter;
    }

    @Override
    public void execute() {
        mShooterSubsystem.setShooterMotorSpeed(0.5, 0.5);
    }

    @Override
    public void end(boolean interrupted) {
        mShooterSubsystem.setShooterMotorSpeed(0.0, 0.0);
        mShooterSubsystem.setAngleShooterMotorSpeed(0.0);
        mShooterSubsystem.setDesiredShooterAngleState(ShooterAngleState.eSpeaker);
        //This should be set to whatever "home" is for the shooter
        //mShooterSubsystem.setDesiredShooterAngleState(ShooterAngleState.eSpeaker);
    }

    @Override
    public boolean isFinished() {
        return !mShooterSubsystem.isNoteLoaded();
    }
    
}
