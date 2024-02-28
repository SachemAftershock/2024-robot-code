package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;

public class NullCommand extends Command {

    final boolean showPrints = false;		



    public NullCommand() {
        addRequirements();
    }   

    @Override
    public void initialize() {
        if (showPrints) System.out.println("NullCommand started.");
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







