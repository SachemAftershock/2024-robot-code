//https://docs.google.com/document/d/1KGTgKLulTAc5xva2maNj7FacMVA5VABR2AX5sK4t-DU/edit?usp=sharing

package frc.robot.enums;


public enum IntakeState {
    eDeployed(-4.0),//TODO: change to eDeployed
    eRetracted(0.0),//TODO: change to eRetracted
    // FIXME. This constant concerns SAFETY, so it needs to be written by a person with
    // knowledge pertaining to what this "safety" even is
    eSafeShooterMovement(0.0); //FIXME might be wrong 
    //Not totally out but far out enought to be safe to move the shooter without fear of getting caught on intake
    
    private double mIntakeEncoderPosition;

    IntakeState(double mIntakeEncoderPosition) {
        this.mIntakeEncoderPosition = mIntakeEncoderPosition;
    }

    public double getDesiredPosition() {
        return mIntakeEncoderPosition;
    }
}
