package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AftershockSubsystem;
import frc.robot.RobotContainer;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPIDCommand extends Command {
    private ShooterSubsystem mShooterSubsystem;
    private RobotContainer mRobotContainer = RobotContainer.getInstance();
    private boolean isIntakeIn;
    private ShooterState mDesiredState;
    public ShooterPIDCommand(ShooterSubsystem mShooterSubsystem) {
        this.mShooterSubsystem = mShooterSubsystem;
        addRequirements(mShooterSubsystem);
    } 

    @Override
    public void initialize() {
        this.mDesiredState = mRobotContainer.getDesiredShooterState();

    }

    @Override
    public void execute() {
        if(mShooterSubsystem.runShooterPID()){
            mRobotContainer.setShooterState(mDesiredState);
        }
    }

    @Override
    public boolean isFinished() {
        return mRobotContainer.getShooterState()==mRobotContainer.getDesiredShooterState();
    }

    @Override
    public void end(boolean interrupted) {
    }
}






