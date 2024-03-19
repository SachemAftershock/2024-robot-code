package frc.robot;

import static frc.robot.Constants.LampConstants.kPdhId;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;

public class LampController {

	private static LampController mInstance;

    private int mNumPulses = 0;
    private double mPulseDurationInSec = 0;
    private double mDelayBetweenPulsesInSec = 0;
    private double mPulseTrainGapInSec = 0; 
    private int mPriority;
    private int mPulseCounter = 0;
    private boolean mPulsePhase = false;
    private final Timer mTimer = new Timer();
    private final PowerDistribution mPDH  = new PowerDistribution(kPdhId, ModuleType.kRev);
    /**
     * 
     * @param numPulses Number of flashes
     * @param pulseDurationInSec How much time it is on in Seconds
     * @param delayBetweenPulsesInSec How much time it is off in Seconds
     * @param pulseTrainGapInSec Time till the next set of flashes
     * @param priority Larger = Higher in queue
     */
    public void setPulse(int numPulses, double pulseDurationInSec, double delayBetweenPulsesInSec, double pulseTrainGapInSec, int priority){
    
        if (!((mNumPulses == numPulses) &&
            (mPulseDurationInSec == pulseDurationInSec) &&
            (mDelayBetweenPulsesInSec == delayBetweenPulsesInSec) &&
            (mPulseTrainGapInSec == pulseTrainGapInSec) && 
            (mPriority >= priority))) {
            mNumPulses = numPulses;
            mPulseDurationInSec = pulseDurationInSec;
            mDelayBetweenPulsesInSec = delayBetweenPulsesInSec;
            mPulseTrainGapInSec = pulseTrainGapInSec;
            mPriority = priority;
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
            mPriority = -1;
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
                 mNumPulses = 0;
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