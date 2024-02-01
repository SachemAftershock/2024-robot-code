package frc.robot.enums;

public enum DonutState {
    eEnteringIntake, //set when b1 is first triggered, slow
    eInIntake, //set when b2 is triggered, stop
    eInShooter, //set when b3 is triggered and b1 is not triggered, stop if in amp
    eExitingShooter; //set when  b4 are trigerred, b3 is not, wait a second and stop
}