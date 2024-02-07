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
    private double velocityOneToOne;

    /**
     * 
     * @param velocityOneToOne velocity between [-1.0, 1.0]
     * @param mIntakeSubsystem
     */
    public IntakeRollerCommand(double velocityOneToOne, IntakeSubsystem mIntakeSubsystem) {
        this.mDesiredState = mIntakeSubsystem.getIntakeState();
        this.mIntakeSubsystem = mIntakeSubsystem;
        addRequirements(mIntakeSubsystem);
        this.velocityOneToOne = velocityOneToOne;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        mIntakeSubsystem.setRollerMotorSpeed(velocityOneToOne);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
