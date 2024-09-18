package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.ShooterAngleState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeArmPositionEnum;

public class AutoAmpScoreCommand extends Command{

    private final IntakeSubsystem mIntake;
    private final ShooterSubsystem mShooterSubsystem;

    public AutoAmpScoreCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
        mIntake = intake;
        mShooterSubsystem = shooter;
    }

    @Override
    public void execute() {

        double shooterSpeed = 0.0;

        // Retract Intake
        mIntake.RetractIntake();

        // If the intake is retracted eject note from intake and ingest note into the shooter
        if (mIntake.getIntakeArmState() == IntakeArmPositionEnum.eRetracted) 
        {
            //System.out.println("Intake is retracted");
            mIntake.ejectNote();
            
            // Ingest note from shooter until beam breaker sees the note
            if (mShooterSubsystem.isNoteLoaded()) 
            {
                //System.out.println("Note loaded");
                shooterSpeed = 0.0;
            } 
            else 
            {
                //System.out.println("Note not loaded");
                shooterSpeed = 0.1;
            }
        }

        // If a note is loaded angle the shooter to amp position
        if (mShooterSubsystem.isNoteLoaded()) 
        {
            //System.out.println("Setting position to amp");
            mShooterSubsystem.setDesiredShooterAngleState(ShooterAngleState.eAmp);
            // mShooterSubsystem.runShooterAngleSetpointChaser();
        }
        else
        {
            mShooterSubsystem.setAngleShooterMotorSpeed(0.0);
        }

        mShooterSubsystem.setShooterMotorSpeed(shooterSpeed, shooterSpeed);

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
        return mShooterSubsystem.getCurrentShooterAngleState() == ShooterAngleState.eAmp;
    }

}
