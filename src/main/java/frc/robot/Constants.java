// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */

public class Constants {

    public static class Intake{
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

    public static class Shooter{
        
        public static class DeviceIDs{
            public static int shooterMotorID = 8;
         }

        public static class ShooterPIDConstants{
            public static double P = 0.0001; //Place holder
            public static double I = 0.0; //Place holder
            public static double D = 0.0; //Place holder
            public static double V = 0.0; //Place holder
            public static double acceleration = 0.0; //Place holder
            public static double cruiseVelocity = 0.0; //Place holder

        }
    }

    public static class ShooterPivot {
        public static final int pivotMotorID = 7;

        public static final double pivotTargetedThreshold = 0.2; //Place holder

        public static final double pivotHeight = 14.9397244; //Pivot axis height above ground when elevator at 0 (in) 
        public static final double shooterLength = 13.342337; //pivot axis to top flywheel axis
        public static final double flyWheelRadius = 2.0;
        public static final double robotMaxHeight = 47; // 1 inch wiggle room

        public static final double pivotLevelOffset = 0.0; //place holder // degrees added to pivot position to find position where level = 0 

        public static final double pivotGearRatio = 60.0;

        public static class PIDController {
            public static final boolean useCodePID = false; // Whether to overwrite PID values in motor
            public static final double P = 0.00001; //Place holder
            public static final double I = 0.0; //Place holder
            public static final double D = 0.000; //Place holder
            public static final double F = 0.01; //Place holder

            public static final double maxVelocity = 30.0; // degrees/second //Place holder
            public static final double maxAcceleration = 60.0; // degrees/second^2 //Place holder

            public static final double peakForwardDutyCycle = 0.4; // Max forward motor power
            public static final double peakReverseDutyCycle = -0.2; // Max reverse power

            public static final double pivotAngleTolerance = 0.0; //Place holder
        }

        public static class Limits {
            public static final double minPivotAngle = 1.5; //PlaceHolder
            public static final double maxPivotAngle = 60.0; //PlaceHolder

            public static final double maxPivotAmp = 10.0; // Max pivot angle when in Amp mode
            public static final double maxPivotEndgame = 1.5;  // Max pivot angle in in Endgame mode

            public static final double pivotCollisionZone = 10.0; //PlaceHolder
            }

        public static class Positions {
            public static final double intakingPiece = 15.0; //Place holder
            public static final double pivotSafeZone = 13.0; //PlaceHolder
        }
    }

    public static class Elevator {
        public static final int leadMotorID = 2; //left
        public static final int followMotor1ID = 3; //left
        public static final int followMotor2ID = 4; //right
        public static final int followMotor3ID = 5; //right

        public static final int leadMotorPDHPort = 0; //place holder

        public static final double gearRatio = (12.0/60.0) * (36.0/64.0); 
        public static final double sprocketPitchDiameter = 1.751; //inches
        public static final double encoderToInches = gearRatio * (sprocketPitchDiameter * Math.PI) * 2; //calculates instage which is double the motion of outer stage

        public static class PIDConstants{
            public static final double kP = 2.5; //placeholder
            public static final double kI = 0.0; //placeholder
            public static final double kD = 0.0; //placeholder
            public static final double kIZone = 0.0; //placeholder
            public static final double kF = 0.025; //Place holder

            public static final double kMinOutput = -0.2; //placeholder
            public static final double kMaxOutput = 0.2; //placeholder

            public static final double SmartMotionMaxVel = 350.0; //placeholder
            public static final double SmartMotionMinVel = -350.0; //placeholder
            public static final double SmartMotionMaxAcc = 500.0; //placeholder
            public static final double SmartMotionAllowedError = 0.1; //placeholder


            public static final double positionTolerance = 0.0; //placeholder

            public static final double integratorRangeMin = -.2; //placeholder (Copied from Rodrigue's Code)
            public static final double integratorRangeMax = .2; //Place holder
        }

        public static final double elevatorHeightToleranceInch = 0.5; //Place holder

        public static class Limits {
            public static final double softStopBottom = 1.0; //bottom soft stop (in) //Place holder
            public static final double softStopTop = 24.5; //bottom soft stop (in) //PlaceHolder
            public static final double hardStopTop = 25.0; //bottom soft stop (in) //Place holder

            public static final double hardStopCurrentLimit = 10.0; // Place Holder

            public static final double elevatorDangerZone = 7.0; //placeholder
            public static final double intakeDangerZone = 2.0; //placeholder

        }

        public static class Positions {
            public static final double intakingPiece = 0.0; //Place holder
            public static final double safeZoneTop = 7.5; //placeholder
        }

        public static class Climb {
            public static final double joyStickDeadBand = 0.2; 
        }
    }
}
