package frc.robot.subsystems;

import frc.lib.AftershockSubsystem;

public class IntakeSubsystem extends AftershockSubsystem  {
    private static IntakeSubsystem mInstance;

    private IntakeSubsystem(){
        System.out.println("INTAKE");
    }
    public synchronized static IntakeSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new IntakeSubsystem();
		}
		return mInstance;
	}

    @Override
	public void initialize() {
    }

    @Override
	public void outputTelemetry() {

	}

}
