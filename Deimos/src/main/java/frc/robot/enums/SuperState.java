//not sure about the function of superstate atm

package frc.robot.enums;

import frc.robot.RobotContainer;

public enum SuperState {//at the end of an amp command, reset current state to null
    eArmAmp(ShooterState.eAmp, IntakeState.eOut),
    eArmSpeakerLeft(ShooterState.eSpeaker, IntakeState.eOut),
    eArmSpeakerRight(ShooterState.eSpeaker, IntakeState.eOut),
    eArmSpeakerCenter(ShooterState.eSpeaker, IntakeState.eOut),
    eArmSpeakerCenter(ShooterState.eSpeaker, IntakeState.eOut);

    SuperState(ShooterState mShooterState, IntakeState mIntakeState){
        this.mShooterState = mShooterState;
        this.mIntakeState = mIntakeState;
    }
    private ShooterState mShooterState;
    private IntakeState mIntakeState;
    public ShooterState getShooterState() {
        return mShooterState;
    }

    public IntakeState getIntakeState() {
        return mIntakeState;
    }
}
  