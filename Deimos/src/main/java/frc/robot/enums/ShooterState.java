package frc.robot.enums;

import frc.robot.RobotContainer;

public enum ShooterState {
    eSpeaker(0, IntakeState.eIn),
    eSafeZone(0, IntakeState.eIn),
    eAmp(0, IntakeState.eIn);
    //TODO: remove paired intake state
    private double angle;
    private IntakeState pairedIntakeState;
    
    ShooterState(double angle, IntakeState pairedIntakeState) {
        this.angle = angle;
    }
    public IntakeState getIntakeState(){
        return pairedIntakeState;
    }
    public double getAngle() {
            return angle;
    }
}
