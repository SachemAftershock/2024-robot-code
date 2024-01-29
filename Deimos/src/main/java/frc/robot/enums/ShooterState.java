package frc.robot.enums;

import frc.robot.RobotContainer;

public enum ShooterState {
    eSpeaker(0, IntakeState.eSpeaker),
    eSafeZone(0, IntakeState.eSafeZone),
    eAmp(0, IntakeState.eAmp);
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