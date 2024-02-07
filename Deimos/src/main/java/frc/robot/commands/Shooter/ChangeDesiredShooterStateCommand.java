package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.AftershockSubsystem;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.IntakePIDCommand;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ChangeDesiredShooterStateCommand extends Command {
    private ShooterSubsystem mShooterSubsystem;
    private RobotContainer mRobotContainer = RobotContainer.getInstance();
    private ShooterState mDesiredState;
    public ChangeDesiredShooterStateCommand(ShooterState mDesiredState, ShooterSubsystem mShooterSubsystem) {
        this.mDesiredState = mDesiredState;
        this.mShooterSubsystem = mShooterSubsystem;
        addRequirements(mShooterSubsystem);
    } 

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        mShooterSubsystem.setDesiredShooterState(mDesiredState);

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().schedule();
    }
}






