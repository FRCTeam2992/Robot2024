// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.leds.Color;

/** Add your docs here. */
public class Constants {
    public static final boolean dataLogging = false;
    public static final boolean debugDashboard = true;

    public static class Vision {
        public static class LimeLight3g {
            public static final double targetAreaThreshold = 0.09;
            // Degrees/sec angular velocity
            public static final double angularVelocityThreshold = 720.0;
            // Meters moved in a single cycle
            public static final double distanceMovedInCycleThreshold = 2.0 / 50.0;
        }
        public static class LimeLight2Plus {
            public static final double targetAreaThreshold = 0.16;
            // Degrees/sec angular velocity
            public static final double angularVelocityThreshold = 360.0;
            // Meters moved in a single cycle
            public static final double distanceMovedInCycleThreshold = 1.0 / 50.0;
        }
        public static final double totalTargetAreaThreshold = Math.min(LimeLight3g.targetAreaThreshold, LimeLight2Plus.targetAreaThreshold);
    }

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
        public static final boolean odometryThread = false;
        public static final int odometryFastRefreshTimeoutMillis = 5;
        public static final int odometrySlowRefreshTimeoutMillis = 100;
        public static final double sensorUpdateRateHz = 50.0; // 200.0
        public static final byte gyroUpdateRateHz = (byte) 50; // 50
        public static final Pose2d zeroPose = new Pose2d(0.0, 0.0, new Rotation2d());

        // Length and Width of the Robot in Meters (Inches: 24.25 x 24.75)
        public static final double swerveWidth = 0.62865;
        public static final double swerveLength = 0.61595;

        // Max Swerve Speed (Velocity Control)
        public static final double swerveMaxSpeed = 5.0; // (Meters per Second)(2 Slow, 5.0 normal)

        // Swerve Wheels and Gear Ratio
        public static final double driveGearRatio = 6.12;// 6.12:1
        public static final double driveWheelDiameter = 0.09927
        ; // 0.098552 (Tread) 0.1016 (Colson)

        // Analog Encoder Offsets (Degrees) - Opposite of Raw Reading - Bevel Gear to
        // Right
        public static final double frontLeftOffset = 34.63; // 31.30; // -174.3;
        public static final double frontRightOffset = 153.37; // 159.79; // 95.0; //90.8
        public static final double rearLeftOffset = -167.52; // -161.00; // 180.6;//170.6
        public static final double rearRightOffset = -19.34; // -20.84; // 28.3;//31.0

        public static class PIDConstants {
            // Swerve Drive PID (Velocity Control)
            public static final double driveP = 0.02;
            public static final double driveI = 0.0;
            public static final double driveD = 0.00;
            public static final double driveV = 0.012;

            // Swerve Turn PIDs
            public static final double turnP = 0.013;
            public static final double turnI = 0.0;
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

        // Drive Rotation P
        public static final double driveRotationP = .005;
        public static final double autoAngleThreshold = 0.3;

        // Swerve Module Translations x=.628/2, y=.616/2
        public static final Translation2d frontLeftLocation = new Translation2d(0.314, 0.308);
        public static final Translation2d frontRightLocation = new Translation2d(0.314, -0.308);
        public static final Translation2d rearLeftLocation = new Translation2d(-0.314, 0.308);
        public static final Translation2d rearRightLocation = new Translation2d(-0.314, -0.308);

        // Swerve Drive Base Radius (Path Following)
        public static final double driveBaseRadius = 0.43984;

        // Swerve Translation Correction PID (Path Following)
        public static final double xyCorrectionP = 2.0;
        public static final double xyCorrectionI = 0.0;
        public static final double xyCorrectionD = 0.0;

        // Swerve Theta Axis Correction PID (Path Following)
        public static final double thetaCorrectionP = 3.0;
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

        // Amp Auto Align
        public static final double maxAmpAutoAlignError = 1.0; //Meters

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
            // All in blue field coordiantes
            public static double FIELD_WIDTH_METERS = 8.02;
            public static double FIELD_LENGTH_METERS = 16.522;
            public static double blueGoalX = 0.2286;
            public static double blueGoalY = 5.43;
            public static double redGoalX = FIELD_LENGTH_METERS - blueGoalX;
            public static double redGoalY = blueGoalY;
            public static Translation2d redGoalTarget = new Translation2d(redGoalX, redGoalY);
            public static Translation2d blueGoalTarget = new Translation2d(blueGoalX, blueGoalY);
            public static double redAmpX = 14.679242;
            public static double blueAmpX = 1.821445;

        }

        public static class StageAngles{
            public static double[] angles = {120.0, -120.0, 0.0, 0.0, 120.0, -120.0}; //position 0 corresponds to id 11, 1 to 12, 2 to 13 ...
        }
    }

    public static class Intake {
        public static final int intakeLeadMotorID = 10;
        public static final int intakeFollowMotorID = 11;
        public static final int intakeBeamBreakID = 0;

        public static class Speeds {
            public static final double intakingPieceSpeed = 0.6;
            public static final double outakingPieceSpeed = -0.4;
        }
    }

    public static class Feeder {
        public static final int feederMotorID = 6;

        public static class Speeds {
            public static final double intakingPieceSpeed = 0.40;
            public static final double outakingPieceSpeed = -0.4;
            public static final double speekerShootingSpeed = 0.5;
            public static final double ampShootingSpeed = -0.65;
            public static final double trapShootingSpeed = -0.65;
        }
    }

    public static class Shooter {

        public static class DeviceIDs {
            public static int shooterMotorID = 8;
            public static int followMotorID = 20;
        }

        public static final double defaultShooterSpeed = 3300; // RPM

        public static final double encoderToFlywheelRotations = 1.0;

        public static class ShooterPIDConstants {
            public static final boolean useCodePID = true; // Whether to overwrite PID values in motor

            public static double P = 0.02; 
            public static double I = 0.0; 
            public static double D = 0.0; 
            public static double V = 0.01006; 
            public static double S = 0.00;
            public static double acceleration = 150.0; // Place holder
            public static double jerk = 0.0;
            // public static double cruiseVelocity = 0.0; // Place holder

        }
    }

    public static class ShooterPivot {
        public static final int pivotMotorID = 7;

        public static final double pivotTargetedThreshold = 1.0; // Place holder

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
            public static final double P = 0.4; // Place holder
            public static final double I = 0.0009; // Place holder
            public static final double D = 0.001; // Place holder
            public static final double G = 0.0225; // Place holder

            public static final double maxVelocity = 70.0; // degrees/second //Place holder
            public static final double maxAcceleration = 500.0; // degrees/second^2 //Place holder
            public static final double jerk = 6500.0;

            public static final double peakForwardDutyCycle = 1.0; // Max forward motor power
            public static final double peakReverseDutyCycle = -1.0; // Max reverse power

            public static final double pivotAngleTolerance = 0.0; // Place holder
        }

        public static class Limits {
            public static final double minPivotAngle = 1.5; // PlaceHolder
            public static final double maxPivotAngle = 60.0; // PlaceHolder

            public static final double maxPivotAmp = 29.0; // Max pivot angle when in Amp mode
            public static final double maxPivotEndgame = 1.5; // Max pivot angle in in Endgame mode

            public static final double pivotCollisionZone = 10.0; // PlaceHolder
        }

        public static class Positions {
            public static final double intakingPiece = 29.0; // Place holder
            public static final double pivotSafeZone = 13.0; // PlaceHolder
        }
    }

    public static class Elevator {
        public static final int leadMotorID = 2; // left
        public static final int followMotor1ID = 3; // left
        public static final int followMotor2ID = 4; // right
        public static final int followMotor3ID = 5; // right

        public static final int leadMotorPDHPort = 0; // place holder

        public static final double gearRatio = (12.0 / 78.0) * (28.0 / 64.0);
        public static final double sprocketPitchDiameter = 1.751; // inches
        public static final double encoderToInches = gearRatio * (sprocketPitchDiameter * Math.PI) * 2; // calculates

        public static class PIDConstants {
            public static final double kP0 = 0.15; // placeholder
            public static final double kI0 = 0.00; // placeholder
            public static final double kD0 = 0.0; //0.001; // placeholder
            public static final double kIZone0 = 0.0; // placeholder
            public static final double kF0 = 0.07; // Place holder

            public static final double kP1 = 0.2; // placeholder
            public static final double kI1 = 0.0015; // placeholder
            public static final double kD1 = 0.0; // placeholder
            public static final double kIZone1 = 0.75; // placeholder
            public static final double kF1 = 0.007; // Place holder
            public static final double kSmartMaxAccel = 0.0001; //rpm/s
            public static final double kSmartMaxVel = 2920; //rpm
            public static final double kSmartMinVel = -500.0; //rpm


            public static final double kMinOutput = -0.6; // placeholder
            public static final double kMaxOutput = 1.0; // placeholder

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
            public static final double softStopBottom = 0.5; // bottom soft stop (in) //Place holder
            public static final double softStopTop = 23.0; // bottom soft stop (in) //PlaceHolder
            public static final double hardStopTop = 24.0; // bottom soft stop (in) //Place holder


            public static final double hardStopCurrentLimit = 10.0; // Place Holder

            public static final double elevatorDangerZone = 7.0; // placeholder
            public static final double intakeDangerZone = 2.0; // placeholder

            public static final double maxElevatorAmp = 23.0; 
            public static final double maxElevatorSpeaker = 6.0; 
        }

        public static class Positions {
            public static final double intakingPiece = 0.0; // Place holder
            public static final double safeZoneTop = 7.5; // placeholder
            public static final double speakerShooting = 0.0;
            public static final double ampScoring = 22.0; // placeholder
            public static final double trapScoring = 20.0; // placeholder
        }

        public static class Climb {
            public static final double joyStickDeadBand = 0.1;

        }
    }

    public static class LEDs {
        public static class Colors { // Free to change as you feel
            public static final Color auto = new Color(255, 0, 255); // 
            public static final Color override = new Color(255, 0, 0); // Red
            public static final Color speaker = new Color(0, 0, 255); // Blue
            public static final Color amp = new Color(255, 160, 0); // Yellow
            public static final Color endgame = new Color(255, 0, 255); // purple
            public static final Color defaultSpeaker = new Color(255, 40, 0); // orange
            public static final Color passing = new Color(255, 40, 40); // Pink

            public static final Color aiming = new Color(210, 200, 180); // white
            public static final Color onTarget = new Color(0, 255, 0); // green
            public static final Color shooting = new Color(10, 255, 255); // aqua

            public static final Color intakeSuccess = new Color(0, 255, 0); // Lime
            public static final Color purple = new Color(210, 75, 230);
            public static final Color yellow = new Color(255, 160, 0);
            public static final Color blue = new Color(0, 0, 255);
            public static final Color red = new Color(255, 0, 0);
            public static final Color orange = new Color(255, 80, 0);
            public static final Color white = new Color(210, 200, 180);
            public static final Color off = new Color(0, 0, 0);
    
        }

        public static final int numberOfIntakingChasers = 4;
        public static final int numberOfChaserChasers = 6;
    }
}
