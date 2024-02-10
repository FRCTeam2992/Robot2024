// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class Constants {
    public static final boolean dataLogging = false;
    public static final boolean debugDashboard = true;

     public static class DrivetrainConstants {
        // Drive Variables
        public static final boolean isFieldCentric = true;
        public static final boolean isVelocityControlled = true;
        public static final boolean isGyroCorrected = true;
        public static final double joystickDeadband = 0.15;
        public static double joystickXYSmoothFactor = 0.5;
        public static double joystickRotationSmoothFactor = 0.5;
        public static double joystickRotationInverseDeadband = 0.14;

        // Length and Width of the Robot in Meters (Inches: 28 x 31.5) 23.75 x 24.75
        public static final double swerveWidth = 0.62865; //0.578;
        public static final double swerveLength = 0.60325; //0.667;

        // Max Swerve Speed (Velocity Control)
        public static final double swerveMaxSpeed = 4.5; // (Meters per Second)(2 Slow, 4.5 normal)

        // Swerve Wheels and Gear Ratio
        public static final double driveGearRatio = 6.75;// 6.75:1
        public static final double driveWheelDiameter = 0.098552; //0.098552;

        // Analog Encoder Offsets (Degrees) - Opposite of Raw Reading - Bevel Gear to
        // Right
        public static final double frontLeftOffset =  26.2; //-174.3;
        public static final double frontRightOffset =  159.7; //95.0; //90.8
        public static final double rearLeftOffset = -161.4; //180.6;//170.6
        public static final double rearRightOffset = -85.4; //28.3;//31.0

        public static class PIDConstants {
            // Swerve Drive PID (Velocity Control)
            public static final double driveP = 0.05;// .05
            public static final double driveI = 0.0;// .0
            public static final double driveD = 0.01;
            public static final double driveV = 0.047;

            // Swerve Turn PIDs
            public static final double turnP = 0.013; // .013
            public static final double turnI = 0.0;// .0
            public static final double turnD = 0.00005;
        }

        public static class AutoScorePIDConstants {
            public static final double scoreP = 3.25;
            public static final double scoreI = 0.001;
            public static final double scoreD = 0.1;

            public static final double scoreCruise = 4.0; // m / sec
            public static final double scoreAccel = 4.0; // m / sec^2
        }

        // Gyro P
        public static final double driveGyroP = 0.005;

        // Gyro balancing constants
        public static final double gyroRollOffset = -1.9; // degrees -- its robot pitch but navx roll
        public static final double pitchTolerance = 2.0; // degrees -- level if Abs() less than this
        public static final double pitchDeltaTolerance = 0.08; // degrees/20ms robot cycle
        public static final double balanceMoveSpeed = 0.75; // m/sec -- max speed to crawl for final balance
        public static final double balanceWaitTimer = 1.0; // How long to wait before declaring balanced
        public static final double balanceP = 0.03;
        public static final double balanceI = 0.0;
        public static final double balanceD = 0.006;

        // Drive Rotation P
        public static final double driveRotationP = .01;
        public static final double autoAngleThreshold = 0.3;

        // Swerve Module Translations x=.591/2 y=.654/2
        public static final Translation2d frontLeftLocation = new Translation2d(0.289, 0.3335);
        public static final Translation2d frontRightLocation = new Translation2d(0.289, -0.3335);
        public static final Translation2d rearLeftLocation = new Translation2d(-0.289, 0.3335);
        public static final Translation2d rearRightLocation = new Translation2d(-0.289, -0.3335);

        // Swerve X Axis Correction PID (Path Following)
        public static final double xCorrectionP = 10.0;
        public static final double xCorrectionI = 0.0;
        public static final double xCorrectionD = 0.0;

        // Swerve Y Axis Correction PID (Path Following)
        public static final double yCorrectionP = 10.0;
        public static final double yCorrectionI = 0.0;
        public static final double yCorrectionD = 0.0;

        // Swerve Theta Axis Correction PID (Path Following)
        public static final double thetaCorrectionP = 150.0;
        public static final double thetaCorrectionI = 0.0;
        public static final double thetaCorrectionD = 0.0;

        // Max Path Following Drive Speeds
        public static final double maxPathFollowingVelocity = 3.0; // (Meters per Second)
        public static final double maxPathFollowingAcceleration = 2; // (Meters per Second Squared)

        // Max Path Following Turn Speeds
        public static final double maxThetaVelocity = 6.28; // (Radians per Second)
        public static final double maxThetaAcceleration = 6.28; // (Radians per Second Squared)

        // Max speeds where its safe to X wheels
        public static final double maxSpeedToX = 0.25; // m/sec
        public static final double maxTurnToX = 20.0; // degrees/sec

        public static class CanIDs {
            public static int frontLeftDrive = 2;
            public static int frontLeftTurn = 3;
            public static int frontRightDrive = 4;
            public static int frontRightTurn = 5;
            public static int rearLeftDrive = 6;
            public static int rearLeftTurn = 7;
            public static int rearRightDrive = 8;
            public static int rearRightTurn = 9;

            public static int frontLeftEncoder = 3;
            public static int frontRightEncoder = 5;
            public static int rearLeftEncoder = 7;
            public static int rearRightEncoder = 9;
        }

        // Field Coordinates
        public static class FieldSize {
            public static double FIELD_WIDTH_METERS = 8.02;
            public static double FIELD_LENGTH_METERS = 16.04;
        }
    }
}
