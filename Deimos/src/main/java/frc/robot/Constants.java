// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.OptionalInt;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.PositionToVelocityProfiler;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

        
    public static final class DriverStationConstants {
        public static DriverStation mDriverStation;
        public static Optional<Alliance> kAllianceColor = DriverStation.getAlliance();
        public static OptionalInt kLocationnumber = DriverStation.getLocation();
        public static void updateAllianceColorAndLocation(){
            kAllianceColor = DriverStation.getAlliance();
            kLocationnumber = DriverStation.getLocation();
        }
    }



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
        public static final double kEjectNoteSpeed = 0.7;

        public static double[] kIntakeArmGains = { 0.4, 0, 0 };

        public static double kIntakeArmMotorSpeed = 0.05;
        public static double kIntakeRollerMotorSpeed = 0.05;

        public static PositionToVelocityProfiler kIntakeAngleDeployedProfiler = (new PositionToVelocityProfiler()
            .setGoal(7.4)
            .addProfileEntry(0.0 , 0.2 , 0.8)
            .addProfileEntry(0.2 , 0.4 , 0.)
            .addProfileEntry(0.4, 0.6 , 0.80)
            .addProfileEntry(0.6 , 0.8 , 0.85)
            .addProfileEntry(0.8 , 1.0 , 0.90)
            .addProfileEntry( 1.0, 1.2 , 0.90)
            .addProfileEntry(1.2 , 1.4 , 0.90)
            .addProfileEntry(1.4 , 1.6 , 0.90)
            .addProfileEntry(1.6, 1.8, 0.4)
            .addProfileEntry(1.8 , 2.0 , 0.8)
            .addProfileEntry(2.0 , 2.2 , 0.)
            .addProfileEntry(2.2 , 2.4 , 0.80)
            .addProfileEntry(2.2 , 2.4 , 0.85)
            .addProfileEntry(2.4 , 2.6 , 0.90)
            .addProfileEntry( 2.6, 2.8 , 0.90)
            .addProfileEntry(2.8 , 3.0 , 0.90)
            .addProfileEntry(3.0 , 3.2 , 0.90)
            .addProfileEntry(3.2, 3.4, 0.4)
            .addProfileEntry(3.4 , 3.6 , 0.8)
            .addProfileEntry(3.6 , 3.8 , 0.)
            .addProfileEntry(3.8 , 4.0 , 0.80)
            .addProfileEntry(4.0 , 4.2 , 0.85)
            .addProfileEntry(4.2 , 4.4 , 0.90)
            .addProfileEntry(4.4 , 4.6 , 0.90)
            .addProfileEntry(4.6 , 4.8 , 0.90)
            .addProfileEntry(4.8 , 5.0 , 0.90)

            .addProfileEntry(5.0 , 5.2 , 0.90)
            
            .addProfileEntry(5.2 , 5.4 , 0.90)
            .addProfileEntry(5.4 , 5.6 , 0.90)
            .addProfileEntry(5.6 , 5.8 , 0.70)
            .addProfileEntry(5.8 , 6.0 , 0.55)
            .addProfileEntry(6.0 , 6.2 , 0.35)
            .addProfileEntry(6.2 , 6.4 , 0.07)
            .addProfileEntry(6.4, 6.6, 0.01)
            .addProfileEntry(6.6, 6.8, -0.07)
            .addProfileEntry(6.8 , 7.0 , 0.90)
            
            .addProfileEntry(7.0 , 7.2 , 0.90)
            .addProfileEntry(7.2 , 7.4 , 0.90)

            // in case it goes too far somehow
        );

    }

    public static class ClimberConstants {
        public static final int kLeftClimberMotorID = 26;
        public static final int kRightClimberMotorID = 25;

        public static final int kLeftLimitSwitchID = 6;
        public static final int kRightLimitSwitchID = 9;

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

        public static final int kBeamBreakerEnterID = 1;
        public static final int kBeamBreakerLeaveID = 8;

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
            .setGoal(97)
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
            .addProfileEntry(85 , 95 , 0.07)
            .addProfileEntry(95 , 100, 0.01)
            .addProfileEntry(100, 105, -0.07)

            // in case it goes too far somehow
            .addProfileEntry(105, 240, -0.15)
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
            .addProfileEntry(80 , 240, -0.15) // just in case it goes past somehow
        );

        /**
         * 130 degrees. When climbing on the chain, we lift the shooter arm all the way up to aim at the trap,
         * then fire backwards.
         */
        public static PositionToVelocityProfiler kShooterAngleTrapProfiler = (new PositionToVelocityProfiler()
            .setGoal(127)
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
            .addProfileEntry(75 , 90 , 0.35)
            .addProfileEntry(90 , 115 , 0.20)
            .addProfileEntry(115 , 125, 0.1)
            .addProfileEntry(125, 130, .01)
            .addProfileEntry(130, 135, -0.01)
            .addProfileEntry(135, 140, -0.07)
            // in case it goes too far somehow
            .addProfileEntry(130, 240, -0.15)
        );

        /**
         * 10 degrees. A bit off, so that notes can be dropped in from the depository
         * NOT INITIALIZED YET
         */
        public static PositionToVelocityProfiler kShooterAngleSafeZoneProfiler = (new PositionToVelocityProfiler()
            .setGoal(10)
            .addProfileEntry(-2, 10, 0.10)
            .addProfileEntry(10, 20, 0.00)
            .addProfileEntry(20, 30, 0.00)
            .addProfileEntry(30, 40, 0.00)
            .addProfileEntry(40, 50, 0.00)
            .addProfileEntry(50, 60, 0.00)
            .addProfileEntry(60, 70, 0.00)
            .addProfileEntry(70, 80, 0.00)

            .addProfileEntry(155, 240, -0.10) // if it goes too far somehow. tho safezone isnt used yet
        );

    }

    public static class DriveConstants {

        public static enum CardinalDirection {
            eX, eY
        }

        public static final double kDriveControllerDeadband = 0.05;
        public static final boolean kSquareAxis = true;

        public static final double[] kDriveAngularGains = { 0.2, 0.0, 0.0 }; // dont use I it sucks - Shreyas
        public static final double[] kDriveLinearGains = { 0.95, 0.0, 0.0 }; // {0.8, 0.02, 0.0}
        public static final double kDt = 0.02;// 0.02;

        public static final double kPX = 1.25;
        public static final double kPY = 1.25;
        public static final double kPTheta = 1; // FIXME made on-the-fly for choreo

        public static final double kAutoRotateEpsilon = 3.0;
        public static final double kLinearDriveTranslationEpsilon = 0.04;// .05
        public static final double kLinearDriveRotationEpsilon = 1.0;
        public static final double kLinearDriveEpsilon = 0.3;

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
