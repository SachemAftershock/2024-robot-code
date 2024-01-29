package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AftershockSubsystem;
import frc.robot.RobotContainer;
import frc.robot.enums.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePIDCommand extends Command {
    private IntakeSubsystem mIntakeSubsystem;
    private RobotContainer mRobotContainer = RobotContainer.getInstance();
    private boolean isIntakeIn;
    private IntakeState mDesiredState;
    public IntakePIDCommand(IntakeSubsystem mIntakeSubsystem) {
        mRobotContainer.setIntakeState(IntakeState.eSpeaker);
        this.mDesiredState = mRobotContainer.getIntakeState();
        this.mIntakeSubsystem = mIntakeSubsystem;
        addRequirements(mIntakeSubsystem);
    } 

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if(mIntakeSubsystem.runIntakePID()){
            mRobotContainer.setIntakeState(mDesiredState);
        }
    }

    @Override
    public boolean isFinished() {
        return mRobotContainer.getIntakeState() == mRobotContainer.getDesiredIntakeState();
    }

    @Override
    public void end(boolean interrupted) {
    }
}






