package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.ShooterAngleState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeArmPositionEnum;

public class ManualAmpScoreCommand extends Command{

    private final IntakeSubsystem mIntake;
    private final ShooterSubsystem mShooter;
    private final BooleanSupplier trigger_Supplier;

    public ManualAmpScoreCommand(IntakeSubsystem intake, ShooterSubsystem shooter, BooleanSupplier triggSupplier) {
        mIntake = intake;
        mShooter = shooter;
        trigger_Supplier = triggSupplier;
    }

    @Override
    public void execute() {

        double shooterSpeed = 0.0;

        // Retract Intake
        mIntake.RetractIntake();

        // If the intake is retracted eject note from intake and ingest note into the shooter
        if (mIntake.getIntakeArmState() == IntakeArmPositionEnum.eRetracted) 
        {
            mIntake.ejectNote();
            
            // Ingest note from shooter until beam breaker sees the note
            if (!(mShooter.isNoteLoaded())) 
            {
                shooterSpeed = 0.2;
            } 
            else 
            {
                shooterSpeed = 0.0;
            }
        }

        // If a note is loaded angle the shooter to amp position
        if (mShooter.isNoteLoaded()) 
        {
            mShooter.setDesiredShooterAngleState(ShooterAngleState.eAmp);
            mShooter.runShooterAngleSetpointChaser();
        }

        if (mShooter.getCurrentShooterAngleState() == ShooterAngleState.eAmp) 
        {
            //May need to invert this
            if (trigger_Supplier.getAsBoolean()) 
            {
                shooterSpeed = 0.2;
            }
        }

        mShooter.setShooterMotorSpeed(shooterSpeed, shooterSpeed);

    }

    @Override
    public void end(boolean interrupted) {
        mShooter.setShooterMotorSpeed(0.0, 0.0);
        //This should be set to whatever "home" is for the shooter
        mShooter.setDesiredShooterAngleState(ShooterAngleState.eSpeaker);
    }

}
