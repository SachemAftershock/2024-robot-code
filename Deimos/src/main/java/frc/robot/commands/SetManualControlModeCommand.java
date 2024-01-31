package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AftershockSubsystem;
import frc.robot.RobotContainer;
import frc.robot.enums.ControlState;
import frc.robot.enums.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;
//comand which sets the manual or automatic control state and also recalls the configbuttonbindings method to reset the controller mappings, because the same mappings are used for different commands over the 2 modes
public class SetManualControlModeCommand extends Command {
    private RobotContainer mRobotContainer = RobotContainer.getInstance();
    private boolean isIntakeIn;
    private IntakeState mDesiredState;
    private IntakeState mCurrentState;
    public SetManualControlModeCommand(boolean isManualModeDesired) {
        if(isManualModeDesired){
            mRobotContainer.setControlState(ControlState.eManualControl);
        } else if (!isManualModeDesired){
            mRobotContainer.setControlState(ControlState.eAutomaticControl);      
        }
        
        
    } 

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        mRobotContainer.configureButtonBindings();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}






