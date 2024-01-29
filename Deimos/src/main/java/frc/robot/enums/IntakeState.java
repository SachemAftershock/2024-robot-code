//https://docs.google.com/document/d/1KGTgKLulTAc5xva2maNj7FacMVA5VABR2AX5sK4t-DU/edit?usp=sharing

package frc.robot.enums;


public enum IntakeState {
    eOut(5),
    eSpeaker(1),// making the assumption that speaker is in stow
    eAmp(1),
    eSafeZone(1),
    eSafeShooterMovement(1);//Not totally out but far out enought to be safe to move the shooter without fear of getting caught on intake
    private double mIntakeEncoderPosition;
    private ShooterState mPairedShooterState;
    IntakeState(double mIntakeEncoderPosition) {
        this.mIntakeEncoderPosition = mIntakeEncoderPosition;
    }
    public double getPosition(){
        return mIntakeEncoderPosition;
    }
}
