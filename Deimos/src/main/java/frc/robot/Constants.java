// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.PositionToVelocityProfiler;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class LampConstants {
        public static int kPdhId = 3;        
    }

    public static final class LimelightConstants {
        public static final String kShooterTableName = "limelight";
        //public static final String kIntakeTableName = "limelight-intake";
    }

    public static class IntakeConstants {
        // public static final double kExampleVariable = 0.05;
        public static final double kIntakeConstraintsMaxVelocity = 1.0;
        public static final double kIntakeConstraintsMaxAcceleration = 1.0;

        public static final int kIntakeArmMotorID = 21;
        public static final int kIntakeRollerMotorID = 20;

        public static final int kExternalBeamBreakerID = 7; // FIXME change these ids pretty pls thx <3
        public static final int kInternalBeamBreakerID = 2; // FIXME change these ids pretty pls thx <3

        public static final int kIntakeLimitSwitchID = 4;

        public static final double kIngestNoteSpeed = -0.4;
        public static final double kEjectNoteSpeed = 0.5;

        public static double[] kIntakeArmGains = { 0.4, 0, 0 };

        public static double kIntakeArmMotorSpeed = 0.05;
        public static double kIntakeRollerMotorSpeed = 0.05;

        public static PositionToVelocityProfiler kIntakeSafeZoneAngleProfiler = (new PositionToVelocityProfiler()
            .setGoal(10)
            .addProfileEntry(0 , 10, 1)
            .addProfileEntry(10, 20, 1)
            .addProfileEntry(10, 20, 1)
            .addProfileEntry(10, 20, 1)
            .addProfileEntry(10, 20, 1)
        );
    }

    public static class ClimberConstants {
        public static final int kLeftClimberMotorID = 25;
        public static final int kRightClimberMotorID = 26;

        public static final double kClimberMotorSpeed = 0.4;
    }

    public static class ShooterConstants {
        // PID
        public static final double[] kAngleGains = { 0.4, 0, 0 };
        public static final double kAngleMaxVelocity = 0.5;
        public static final double kAngleMaxAcceleration = 0.5;
        public static final double kShooterAngleEpsilon = .1;

        public static final double[] kShooterGains = { 0.4, 0, 0 };
        public static final double kConstraintsMaxVelocity = 0.5;
        public static final double kShooterConstraintsMaxAcceleration = 0.5;
        public static final double kShooterAngleSetPWhenBelowEpsilon = 4;
        public static final double kLeftShootMotorSpeed = 0.05; // Derive empirically
        public static final double kRightShootMotorSpeed = 0.05; // Derive empirically
        public static final double kShooterEncoderSetPoint = 0.5; // Derive empirically

        // IDs
        public static final int kLeftShootMotorID = 24;
        public static final int kRightShootMotorID = 23;
        public static final int kAngleShootMotorID = 22;

        public static final int kAngleEncoderID1 = 4; // TODO 4 to 8 are placeholder numbers
        public static final int kAngleEncoderID2 = 5; // TODO
        public static final int kAngleCANcoderID = 13;

        public static final int kBeamBreakerEnterID = 0; // TODO
        public static final int kBeamBreakerLeaveID = 1; // TODO

        public static final int kShooterLimitSwitchID = 5;

        public static final int kClimberArmMotorID = 0; // TODO
        public static final int kClimberRollerMotorID = 0; // TODO

        // hardcoded speeds

        /**
         * WARNING: Currently between [-1.0, 1.0]
         */
        public static double kArmAmpRollerVelocity = 0.5;

        // TODO are we going to make the left motor and right motors different?
        public static double kShootMotorShootingVelocity = 1.0;

        // shooter angle arm position to velocity profiles (aka lookup tables)
        /**
         * 95 degrees. Move the shooter arm up to dump note into amp
         */
        public static PositionToVelocityProfiler kShooterAngleAmpProfiler = (new PositionToVelocityProfiler()
            .setGoal(95)
            .addProfileEntry(-2, 10, 0.4)
            .addProfileEntry(10 , 15 , 0.60)
            .addProfileEntry(15 , 20 , 0.70)
            .addProfileEntry(20 , 25 , 0.80)
            .addProfileEntry(25 , 30 , 0.85)
            .addProfileEntry(30 , 35 , 0.90)
            .addProfileEntry(35 , 40 , 0.90)
            .addProfileEntry(40 , 45 , 0.90)
            .addProfileEntry(45 , 50 , 0.90)

            .addProfileEntry(50 , 55 , 0.90)
            
            .addProfileEntry(55 , 60 , 0.90)
            .addProfileEntry(60 , 65 , 0.90)
            .addProfileEntry(65 , 70 , 0.70)
            .addProfileEntry(70 , 75 , 0.55)
            .addProfileEntry(75 , 85 , 0.35)
            .addProfileEntry(85 , 95 , 0.15)
            .addProfileEntry(95 , 100, 0.01)
            // .addProfileEntry(90, 100, 0.01)
        );

        /**
         * 0 degrees. Puts the shooter arm back all the way down
         */
        public static PositionToVelocityProfiler kShooterAngleSpeakerProfiler = (new PositionToVelocityProfiler()
            .setGoal(0)
            .addProfileEntry(-2, 2, 0)
            .addProfileEntry(2  , 10 , -0.2)
            .addProfileEntry(10 , 20 , -0.3)
            .addProfileEntry(20 , 40 , -0.4)
            .addProfileEntry(40 , 60 , -0.4)
            .addProfileEntry(60 , 100, -0.3)
            .addProfileEntry(80 , 200, -0.2) // just in case it goes past somehow
        );

        /**
         * 130 degrees. When climbing on the chain, we lift the shooter arm all the way up to aim at the trap,
         * then fire backwards.
         */
        public static PositionToVelocityProfiler kShooterAngleTrapProfiler = (new PositionToVelocityProfiler()
            .setGoal(150)
            .addProfileEntry(-2,10,0.1)
            .addProfileEntry(10 ,20 ,0.25)
            .addProfileEntry(20 ,30 ,0.40)
            .addProfileEntry(30 ,40 ,0.55)
            .addProfileEntry(40 ,50 ,0.60)
            .addProfileEntry(50 ,60 ,0.70)
            .addProfileEntry(60 ,70 ,0.70)
            .addProfileEntry(70 ,80 ,0.70)
            .addProfileEntry(80 ,90 ,0.70)
            .addProfileEntry(90 ,100,0.70)
            .addProfileEntry(100,110,0.70)
            .addProfileEntry(110,120,0.60)
            .addProfileEntry(120,130,0.50)
            .addProfileEntry(120,130,0.40)
            .addProfileEntry(130,140,0.25)
            .addProfileEntry(140,147,0.10)
            .addProfileEntry(147,155,0.01)
            // if it overshoots (not possible since we will be trapping with our back to the wall), move back to normal spot
            .addProfileEntry(155, 200, -0.10) 
        );

        /**
         * 10 degrees. A bit off, so that notes can be dropped in from the depository
         * NOT INITIALIZED YET
         */
        public static PositionToVelocityProfiler kShooterAngleSafeZoneProfiler = (new PositionToVelocityProfiler()
            .setGoal(51)
            .addProfileEntry(-2, 10, 0.10)
            .addProfileEntry(10, 20, 0.20)
            .addProfileEntry(20, 30, 0.30)
            .addProfileEntry(30, 40, 0.20)
            .addProfileEntry(40, 50, 0.10)
            .addProfileEntry(50, 55, 0.01)
            .addProfileEntry(55, 60, -0.1)
            .addProfileEntry(60, 70, -0.20)
            .addProfileEntry(70, 200, -0.30)
        );

    }

    public static class DriveConstants {

        public static enum CardinalDirection {
            eX, eY
        }

        public static final int kReinitializeFieldOrientJoystickButton = 7;

        public static final double kDriveControllerDeadband = 0.05;
        public static final boolean kSquareAxis = true;

        public static final double[] kDriveAngularGains = { 0.2, 0.0, 0.0 }; // dont use I it sucks - Shreyas
        public static final double[] kDriveLinearGains = { 1.8, 0.03, 0.0 }; // {0.8, 0.02, 0.0}
        public static final double kDt = 0.02;// 0.02;

        public static final double kPX = 1.25;
        public static final double kPY = 1.25;

        public static final double kAutoRotateEpsilon = 3.0;
        public static final double kLinearDriveTranslationEpsilon = 0.04;// .05
        public static final double kLinearDriveRotationEpsilon = 2.0;

        public static final double kDrivetrainTrackwidthMeters = 0.5461;
        public static final double kDrivetrainWheelbaseMeters = 0.5461;

        // angles in radians.
        // to convert from degrees to radians multiply by pi/180
        public static final double kFrontLeftSteerOffset = (10.0 * Math.PI) / 180.0;
        public static final double kFrontRightSteerOffset = (-150.0 * Math.PI) / 180.0;
        public static final double kBackLeftSteerOffset = (126.0 * Math.PI) / 180.0;
        public static final double kBackRightSteerOffset = (-50.0 * Math.PI) / 180.0;

        private static final double kMk4L1DriveReduction = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
        private static final double kMk4WheelDiameter = 0.10033;

        public static final double kMaxVelocityMetersPerSecond = (6380.0 / 60.0 * kMk4L1DriveReduction
                * kMk4WheelDiameter * Math.PI) * 1.0;// (6380.0 / 60.0 * kMk4L1DriveReduction * kMk4WheelDiameter *
                                                     // Math.PI) * 1.0

        // TODO: Change
        public static final double kMaxAccelerationMetersPerSecondSquared = kMaxVelocityMetersPerSecond * 1.0; // kMaxVelocityMetersPerSecond
                                                                                                               // * 1.0
        public static final double kMaxRotationalVelocityMetersPerSecond = 3.0;

        /**
         * 7.768981298515446
         */
        public static final double kMaxAngularVelocityRadiansPerSecond = kMaxRotationalVelocityMetersPerSecond /
                Math.hypot(kDrivetrainTrackwidthMeters / 2.0, kDrivetrainWheelbaseMeters / 2.0); // kMaxVelocityMetersPerSecond
                                                                                                 // /Math.hypot(kDrivetrainTrackwidthMeters
                                                                                                 // / 2.0,
                                                                                                 // kDrivetrainWheelbaseMeters
                                                                                                 // / 2.0)

        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;
    }

}
