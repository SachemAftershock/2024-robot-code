package frc.robot.commands.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AftershockSubsystem;
import frc.robot.RobotContainer;
import frc.robot.enums.IntakeState;
import frc.robot.enums.ShooterState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualShooterAngleCommand extends Command {
    private ShooterSubsystem mShooterSubsystem;
    private RobotContainer mRobotContainer = RobotContainer.getInstance();
    private ShooterState mDesiredState;
    private ProfiledPIDController mShooterAnglePIDController = mShooterSubsystem.getShooterAnglePIDController();
    private double speed;
    public ManualShooterAngleCommand(ShooterSubsystem mShooterSubsystem, double speed) {
        mRobotContainer.setIntakeState(IntakeState.eRetracted);
        this.mShooterSubsystem = mShooterSubsystem;
        addRequirements(mShooterSubsystem);
        this.speed = speed;
    } 

    @Override
    public void initialize() {
    
    }

    @Override
    public void execute() {
        mShooterSubsystem.manualJogShooter(speed);
        mShooterAnglePIDController.calculate(speed);
    }

    @Override
    public boolean isFinished() {
        return mRobotContainer.getShooterState()==mRobotContainer.getDesiredShooterState();
    }

    @Override
    public void end(boolean interrupted) {
    }
}






