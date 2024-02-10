// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static class IntakeConstants {
        // public static final double kExampleVariable = 0.05;

        public static final int kIntakeArmMotorID = 21;
        public static final int kIntakeRollerMotorID = 20;

        public static final int kExternalBeamBreakerID = 2; // FIXME change these ids pretty pls thx <3
        public static final int kInternalBeamBreakerID = 3;// FIXME change these ids pretty pls thx <3

        public static final int kIntakeLimitSwitchID = 4;

        public static final double kIngestNoteSpeed = 0.4;
        public static final double kEjectNoteSpeed = 0.5;

        public static double[] kIntakeArmGains = { 0.05, 0.0, 0.0 };

        public static double kIntakeArmMotorSpeed = 0.05;
        public static double kIntakeRollerMotorSpeed = 0.05;

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

        public static final int kBeamBreakerEnterID = 6; // TODO
        public static final int kBeamBreakerLeaveID = 7;

        public static final int kShooterLimitSwitchID = 8; // TODO

        public static final int kClimberArmMotorID = 0; // TODO
        public static final int kClimberRollerMotorID = 0; // TODO

        // hardcoded speeds

        /**
         * WARNING: Currently between [-1.0, 1.0]
         */
        public static double kArmAmpRollerVelocity = 0.5;

    }

    public static class DriveConstants {

        public static enum CardinalDirection {
            eX, eY
        }

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

        public static final double kMaxAngularVelocityRadiansPerSecond = kMaxRotationalVelocityMetersPerSecond /
                Math.hypot(kDrivetrainTrackwidthMeters / 2.0, kDrivetrainWheelbaseMeters / 2.0); // kMaxVelocityMetersPerSecond
                                                                                                 // /Math.hypot(kDrivetrainTrackwidthMeters
                                                                                                 // / 2.0,
                                                                                                 // kDrivetrainWheelbaseMeters
                                                                                                 // / 2.0)

        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;
    }

}
