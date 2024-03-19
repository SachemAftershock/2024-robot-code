package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import static frc.robot.Constants.LampConstants.*;

public class LampController {

	private static LampController mInstance;

    private int mNumPulses = 0;
    private double mPulseDurationInSec = 0;
    private double mDelayBetweenPulsesInSec = 0;
    private double mPulseTrainGapInSec = 0; 
    private boolean mOnceOnly = true;

    private int mPulseCounter = 0;
    private boolean mPulsePhase = false;
    private final Timer mTimer = new Timer();
    private final PowerDistribution mPDH  = new PowerDistribution(kPdhId, ModuleType.kRev);

    public void setPulse(int numPulses, double pulseDurationInSec, double delayBetweenPulsesInSec, double pulseTrainGapInSec, boolean onceOnly){
    
        if (!((mNumPulses == numPulses) &&
            (mPulseDurationInSec == pulseDurationInSec) &&
            (mDelayBetweenPulsesInSec == delayBetweenPulsesInSec) &&
            (mPulseTrainGapInSec == pulseTrainGapInSec) && 
            (mOnceOnly = onceOnly))) {

            mNumPulses = numPulses;
            mPulseDurationInSec = pulseDurationInSec;
            mDelayBetweenPulsesInSec = delayBetweenPulsesInSec;
            mPulseTrainGapInSec = pulseTrainGapInSec;
            mOnceOnly = onceOnly;

            mPulseCounter = 0;
            mPulsePhase = false;
            mPDH.setSwitchableChannel(mPulsePhase);
            mTimer.restart();
     }
    }

    public void run() {
        if (mNumPulses == 0) {
            mPulseCounter = 0;
            mPulsePhase = false;
            mPDH.setSwitchableChannel(mPulsePhase);
            mTimer.restart();
        } else if (mPulseCounter < mNumPulses) {
             if(mPulsePhase) {
                if (mTimer.advanceIfElapsed(mPulseDurationInSec)) {
                    mPulsePhase = !mPulsePhase;
                    mPDH.setSwitchableChannel(mPulsePhase);
                    mTimer.restart();            
                    mPulseCounter++;    
                }      
             } else {
                if (mTimer.advanceIfElapsed(mDelayBetweenPulsesInSec)) {
                    mPulsePhase = !mPulsePhase;
                    mPDH.setSwitchableChannel(mPulsePhase);
                    mTimer.restart();
                }
            }            
        } else {
            mPulsePhase = false;
            mPDH.setSwitchableChannel(mPulsePhase);
            if (mTimer.advanceIfElapsed(mPulseTrainGapInSec)) {
                if (mOnceOnly) {
                    mNumPulses = 0;
                } else {
                    mPulseCounter = 0;
                    mTimer.restart(); 
                }
            }
        }  
    }

    public synchronized static LampController getInstance() {
        if (mInstance == null) {
            mInstance = new LampController();
        }
        return mInstance;
    }
   
}