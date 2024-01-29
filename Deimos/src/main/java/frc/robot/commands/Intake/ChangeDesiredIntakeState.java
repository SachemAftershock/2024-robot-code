package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AftershockSubsystem;
import frc.robot.RobotContainer;
import frc.robot.enums.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;

public class ChangeDesiredIntakeState extends Command {
    private IntakeSubsystem mIntakeSubsystem;
    private RobotContainer mRobotContainer = RobotContainer.getInstance();
    private boolean isIntakeIn;
    private IntakeState mDesiredState;
    private IntakeState mCurrentState;
    public ChangeDesiredIntakeState(IntakeState desiredState, IntakeSubsystem mIntakeSubsystem) {
        this.mDesiredState = desiredState;
        this.mIntakeSubsystem = mIntakeSubsystem;
        addRequirements(mIntakeSubsystem);
    } 

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        mRobotContainer.setDesiredIntakeState(mDesiredState);;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}






