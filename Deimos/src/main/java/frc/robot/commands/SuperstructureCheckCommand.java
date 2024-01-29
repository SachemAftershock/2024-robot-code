//Not entirely sur of the use of this for now, maybe to check all sensors and make sure states are correct

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.AftershockSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;

public class SuperstructureCheckCommand extends Command {
    private SuperstructureSubsystem mSuperstructureSubsystem;
    private boolean isIntakeIn;
    public SuperstructureCheckCommand(SuperstructureSubsystem mSuperstructureSubsystem) {
        this.mSuperstructureSubsystem = mSuperstructureSubsystem;
        addRequirements(mSuperstructureSubsystem);
    } 

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
		
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}






