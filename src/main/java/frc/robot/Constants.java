// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class Constants {
    public static final boolean dataLogging = true;
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

        // Odometry variables
        public static final boolean odometryThread = true;
        public static final int odometryFastRefreshTimeoutMillis = 5;
        public static final int odometrySlowRefreshTimeoutMillis = 100;
        public static final double sensorUpdateRateHz = 200.0;
        public static final byte gyroUpdateRateHz = (byte) 200;
        public static final Pose2d zeroPose = new Pose2d(0.0, 0.0, new Rotation2d());

        // Length and Width of the Robot in Meters (Inches: 28 x 31.5) 23.75 x 24.75
        public static final double swerveWidth = 0.62865; // 0.578;
        public static final double swerveLength = 0.60325; // 0.667;

        // Max Swerve Speed (Velocity Control)
        public static final double swerveMaxSpeed = 4.5; // (Meters per Second)(2 Slow, 4.5 normal)

        // Swerve Wheels and Gear Ratio
        public static final double driveGearRatio = 6.12;// 6.12:1
        public static final double driveWheelDiameter = 0.1016; // 0.098552 (Tread) 0.1016 (Colson)

        // Analog Encoder Offsets (Degrees) - Opposite of Raw Reading - Bevel Gear to
        // Right
        public static final double frontLeftOffset = 26.9; // -174.3;
        public static final double frontRightOffset = 159.3; // 95.0; //90.8
        public static final double rearLeftOffset = -160.4; // 180.6;//170.6
        public static final double rearRightOffset = -24.08; // 28.3;//31.0

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
        public static class Field {
            public static double FIELD_WIDTH_METERS = 8.02;
            public static double FIELD_LENGTH_METERS = 16.522;
            public static double goalX = 0.363;
            public static double blueGoalY = 5.43;
            public static double redGoalY = 2.66;
        }

        public static class StageAngles{
            public static double[] angles = {120.0, -120.0, 0.0, 0.0, 120.0, -120.0}; //position 0 corresponds to id 11, 1 to 12, 2 to 13 ...
        }
    }

    public static class Intake {
        public static final int intakeLeadMotorID = 10;
        public static final int intakeFollowMotorID = 11;

        public static class Speeds {
            public static final double intakingPieceSpeed = 0.4;
            public static final double outakingPieceSpeed = -0.4;
        }
    }

    public static class Feeder {
        public static final int feederMotorID = 6;

        public static class Speeds {
            public static final double intakingPieceSpeed = 0.4;
            public static final double outakingPieceSpeed = -0.4;
            public static final double speekerShootingSpeed = 0.5;
            public static final double ampShootingSpeed = -0.5;
            public static final double trapShootingSpeed = -0.5;
        }
    }

    public static class Shooter {

        public static class DeviceIDs {
            public static int shooterMotorID = 8;
        }

        public static class ShooterPIDConstants {
            public static final boolean useCodePID = true; // Whether to overwrite PID values in motor

            public static double P = 0.0005; // Place holder
            public static double I = 0.0; // Place holder
            public static double D = 0.0; // Place holder
            public static double V = 0.01; // Place holder
            public static double S = 0.01;
            public static double acceleration = 100; // Place holder
            public static double jerk = 400;
            // public static double cruiseVelocity = 0.0; // Place holder

        }
    }

    public static class ShooterPivot {
        public static final int pivotMotorID = 7;

        public static final double pivotTargetedThreshold = 0.2; // Place holder

        public static final double pivotHeight = 14.9397244; // Pivot axis height above ground when elevator at
                                                             // 0 (in)
        public static final double shooterLength = 13.342337; // pivot axis to top flywheel axis
        public static final double flyWheelRadius = 2.0;
        public static final double robotMaxHeight = 47; // 1 inch wiggle room

        public static final double pivotLevelOffset = 0.0; // place holder // degrees added to pivot position to
                                                           // find
                                                           // position where level = 0

        public static final double pivotGearRatio = 60.0;

        public static class PIDController {
            public static final boolean useCodePID = true; // Whether to overwrite PID values in motor
            public static final double P = 0.2; // Place holder
            public static final double I = 0.02; // Place holder
            public static final double D = 0.001; // Place holder
            public static final double G = 0.0225; // Place holder

            public static final double maxVelocity = 48.0; // degrees/second //Place holder
            public static final double maxAcceleration = 480.0; // degrees/second^2 //Place holder
            public static final double jerk = 4000.0;

            public static final double peakForwardDutyCycle = 1.0; // Max forward motor power
            public static final double peakReverseDutyCycle = -1.0; // Max reverse power

            public static final double pivotAngleTolerance = 0.0; // Place holder
        }

        public static class Limits {
            public static final double minPivotAngle = 1.5; // PlaceHolder
            public static final double maxPivotAngle = 60.0; // PlaceHolder

            public static final double maxPivotAmp = 20.0; // Max pivot angle when in Amp mode
            public static final double maxPivotEndgame = 8.0; // Max pivot angle in in Endgame mode

            public static final double pivotCollisionZone = 10.0; // PlaceHolder
        }

        public static class Positions {
            public static final double intakingPiece = 15.0; // Place holder
            public static final double pivotSafeZone = 13.0; // PlaceHolder
        }
    }

    public static class Elevator {
        public static final int leadMotorID = 2; // left
        public static final int followMotor1ID = 3; // left
        public static final int followMotor2ID = 4; // right
        public static final int followMotor3ID = 5; // right

        public static final int leadMotorPDHPort = 0; // place holder

        public static final double gearRatio = (12.0 / 60.0) * (36.0 / 64.0);
        public static final double sprocketPitchDiameter = 1.751; // inches
        public static final double encoderToInches = gearRatio * (sprocketPitchDiameter * Math.PI) * 2; // calculates

        public static class PIDConstants {
            public static final double kP0 = 0.1; // placeholder
            public static final double kI0 = 0.0; // placeholder
            public static final double kD0 = 0.0001; // placeholder
            public static final double kIZone0 = 0.0; // placeholder
            public static final double kF0 = 0.035; // Place holder

            public static final double kP1 = 0.03; // placeholder
            public static final double kI1 = 0.0; // placeholder
            public static final double kD1 = 0.0; // placeholder
            public static final double kIZone1 = 0.0; // placeholder
            public static final double kF1 = 0.0; // Place holder

            public static final double kMinOutput = -0.4; // placeholder
            public static final double kMaxOutput = 0.55; // placeholder

            public static final double SmartMotionMaxVel = 350.0; // placeholder
            public static final double SmartMotionMinVel = -350.0; // placeholder
            public static final double SmartMotionMaxAcc = 5.0; // placeholder
            public static final double SmartMotionAllowedError = 1.0; // placeholder

            public static final double positionTolerance = 0.0; // placeholder

            public static final double integratorRangeMin = -.2; // placeholder (Copied from Rodrigue's Code)
            public static final double integratorRangeMax = .2; // Place holder
        }

        public static final double elevatorHeightToleranceInch = 0.5; // Place holder

        public static class Limits {
            public static final double softStopBottom = 1.0; // bottom soft stop (in) //Place holder
            public static final double softStopTop = 23.0; // bottom soft stop (in) //PlaceHolder
            public static final double hardStopTop = 24.0; // bottom soft stop (in) //Place holder


            public static final double hardStopCurrentLimit = 10.0; // Place Holder

            public static final double elevatorDangerZone = 7.0; // placeholder
            public static final double intakeDangerZone = 2.0; // placeholder

            public static final double maxElevatorAmp = 23.0; // Max pivot angle when in Amp mode
            public static final double maxElevatorSpeaker = 23.0; // Max pivot angle in in Endgame mode
        }

        public static class Positions {
            public static final double intakingPiece = 0.0; // Place holder
            public static final double safeZoneTop = 7.5; // placeholder
        }

        public static class Climb {
            public static final double joyStickDeadBand = 0.1;

        }
    }
}
