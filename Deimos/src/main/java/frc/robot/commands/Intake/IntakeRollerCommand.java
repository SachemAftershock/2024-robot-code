package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AftershockSubsystem;
import frc.robot.RobotContainer;
import frc.robot.enums.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRollerCommand extends Command {
    private IntakeSubsystem mIntakeSubsystem;
    private RobotContainer mRobotContainer = RobotContainer.getInstance();
    private boolean isIntakeIn;
    private IntakeState mDesiredState;
    private double velocity;
    
    public IntakeRollerCommand(double velocity, IntakeSubsystem mIntakeSubsystem) {
        this.mDesiredState = mRobotContainer.getIntakeState();
        this.mIntakeSubsystem = mIntakeSubsystem;
        addRequirements(mIntakeSubsystem);
        this.velocity = velocity;
    } 


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        mIntakeSubsystem.setRollerMotorSpeed(velocity);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}






