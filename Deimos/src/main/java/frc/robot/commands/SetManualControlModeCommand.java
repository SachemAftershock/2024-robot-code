package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AftershockSubsystem;
import frc.robot.RobotContainer;
import frc.robot.enums.ControlState;
import frc.robot.enums.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;

public class SetManualControlModeCommand extends Command {
    private RobotContainer mRobotContainer = RobotContainer.getInstance();
    private boolean isIntakeIn;
    private IntakeState mDesiredState;
    private IntakeState mCurrentState;
    public SetManualControlModeCommand(boolean isManualModeDesired) {
        if(isManualModeDesired){
            mRobotContainer.setControlState(ControlState.eManualControl);
            mRobotContainer.configureButtonBindings();
        } else if (!isManualModeDesired){
            mRobotContainer.setControlState(ControlState.eAutomaticControl);
            mRobotContainer.configureButtonBindings();
        }
        
        
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






