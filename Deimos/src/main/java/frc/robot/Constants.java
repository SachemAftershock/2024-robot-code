// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class IntakeConstants {
        //public static final double kExampleVariable = 0.05;
        public static final int mConstraintsMaxVelocity = 0;
        public static final int mConstraintsMaxAcceleration = 0;

        public static final int mIntakeArmMotorID = 0;
        public static final int mIntakeRollerMotorID = 1;

        public static final int mExternalBeamBreakerID = 0; // FIXME change these ids pretty pls thx <3
        public static final int mInternalBeamBreakerID = 0;// FIXME change these ids pretty pls thx <3

        public static final double mInjestDonutSpeed = 0.4;
        public static final double mEjectDonutSpeed = 0.5;

        public static double[] mIntakeArmGains = {0.4,0,0};
    }

    public static class ClimberConstants{
        public static final double kExampleVariable = 0.05;
    }

    public static class ShooterConstants{
        public static final double kExampleVariable = 0.05; 
        // PID
        public static final double[] mAngleGains= {0.4,0,0};
        public static final double mAngleMaxVelocity = 0.5;
        public static final double mAngleConstraintsMaxAcceleration = 0.5;

        public static final double[] mShooterGains = {0.4,0,0};
        public static final double mConstraintsMaxVelocity = .5;
        public static final double mShooterConstraintsMaxAcceleration = .5;
        
        public static final double mLeftShootMotorSpeed = 0.05; //Derive empirically 
	    public static final double mRightShootMotorSpeed = 0.05; //Derive empirically
        public static final double mShooterEncoderSetPoint = 0.5; //Derive empirically

        // IDs
        public static final int mAngleEncoderID1 = 0; 
	    public static final int mAngleEncoderID2 = 1;
        public static final int mLeftShootMotorID = 0;
        public static final int mRightShootMotorID = 1;
	    public static final int mAngleShootMotorID = 2;
         public static final int mBeamBreakerEnterID = 0;
         public static final int mBeamBreakerLeaveID = 1;

    }
    
    public static class DriveConstants {

        public static enum CardinalDirection {
            eX, eY
        }

        public static final double kDriveControllerDeadband = 0.05;
        public static final boolean kSquareAxis = true; 

        
        


        public static final double[] kDriveAngularGains = {0.0125, 0.0, 0.0}; //dont use I it sucks - Shreyas
        public static final double[] kDriveLinearGains = {1.8, 0.01, 0.0};
        public static final double kDt = 0.02;

        public static final double kPX = 1.25;
        public static final double kPY = 1.25;

        public static final double kAutoRotateEpsilon = 3.0;
        public static final double kLinearDriveEpsilon = 0.01;//.05
        
        
        public static final double kDrivetrainTrackwidthMeters = 0.5461;
        public static final double kDrivetrainWheelbaseMeters = 0.5461;
        
        // angles in radians. 
        // to convert from degrees to radians multiply by pi/180 
        public static final double kFrontLeftSteerOffset = (10.0 * Math.PI) / 180.0;
        public static final double kFrontRightSteerOffset = (-85.0 * Math.PI) / 180.0;
        public static final double kBackLeftSteerOffset = (126.0 * Math.PI) / 180.0;
        public static final double kBackRightSteerOffset = (-52.0 * Math.PI) / 180.0;


        private static final double kMk4L1DriveReduction = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
        private static final double kMk4WheelDiameter = 0.10033;

        
        public static final double kMaxVelocityMetersPerSecond = (6380.0 / 60.0 * kMk4L1DriveReduction * kMk4WheelDiameter * Math.PI) * 0.5;// 6380.0 / 60.0 *kMk4L1DriveReduction * kMk4WheelDiameter * Math.PI

        //TODO: Change to an empirically measured value (currently this is just a random multiplication)
        public static final double kMaxAccelerationMetersPerSecondSquared = (kMaxVelocityMetersPerSecond * 0.5); // previously 0.25

        public static final double kMaxAngularVelocityRadiansPerSecond = kMaxVelocityMetersPerSecond /
        Math.hypot(kDrivetrainTrackwidthMeters / 2.0, kDrivetrainWheelbaseMeters / 2.0);
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;
    }



}
