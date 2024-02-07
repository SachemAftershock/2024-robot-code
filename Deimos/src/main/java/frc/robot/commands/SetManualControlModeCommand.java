package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AftershockSubsystem;
import frc.robot.RobotContainer;
import frc.robot.enums.ControlState;
import frc.robot.enums.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;

//command which sets the manual or automatic control state
//and also recalls the configbuttonbindings method to reset the controller mappings,
//because the same mappings are used for different commands over the 2 modes
public class SetManualControlModeCommand extends Command {
    private boolean isIntakeIn;
    private IntakeState mDesiredState;
    private IntakeState mCurrentState;
    private IntakeSubsystem mIntakeSubsystem = IntakeSubsystem.getInstance();

    public SetManualControlModeCommand(boolean isManualModeDesired) {
        if (isManualModeDesired) {
            // mIntakeSubsystem.setControlState(ControlState.eManualControl);
        } else {
            // mIntakeSubsystem.setControlState(ControlState.eSemiAutoControl);
        }

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
