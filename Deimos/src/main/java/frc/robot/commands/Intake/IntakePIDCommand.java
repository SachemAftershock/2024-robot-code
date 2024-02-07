package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AftershockSubsystem;
import frc.robot.RobotContainer;
import frc.robot.enums.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePIDCommand extends Command {
    private IntakeSubsystem mIntakeSubsystem;
    // private RobotContainer mRobotContainer = RobotContainer.getInstance();
    private boolean isIntakeIn;
    private IntakeState mDesiredState;
    public IntakePIDCommand(IntakeSubsystem mIntakeSubsystem) {
        this.mIntakeSubsystem = mIntakeSubsystem;
        addRequirements(mIntakeSubsystem);
    } 

    @Override
    public void initialize() {
        this.mDesiredState = mIntakeSubsystem.getIntakeState();

    }

    @Override
    public void execute() {
        mIntakeSubsystem.runIntakePID();
    }

    @Override
    public boolean isFinished() {
        return mIntakeSubsystem.getIntakeState() == mIntakeSubsystem.getDesiredIntakeState();
    }

    @Override
    public void end(boolean interrupted) {
    }
}






