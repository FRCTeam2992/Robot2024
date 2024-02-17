// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */

public class Constants {

    public static class Intake{
        public static final int intakeLeadMotorID = 14;
        public static final int intakeFollowMotorID = 18;
    }

    public static class Feeder {
        public static final int feederMotorID = 17;
    }

    public static class Shooter{
        
        public static class DeviceIDs{
            public static int shooterMotorID = 15;
         }

        public static class ShooterPIDConstants{
            public static double P = 0.0;
            public static double I = 0.0;
            public static double D = 0.0;
            public static double V = 0.0;
            public static double acceleration = 0.0;
            public static double cruiseVelocity = 0.0;

        }
    }

    public static class ShooterPivot {
        public static final int pivotMotorID = 16;

        public static final double pivotTargetedThreshold = 5.0;

        public static class PIDController {
            public static final double P = 0.0;
            public static final double I = 0.0;
            public static final double D = 0.0;
            public static final double F = 0.0;

            public static final double pivotAngleTolerance = 0.0;
        }

        public static class Limits {
            public static final double minPivotAngle = 0.0;
            public static final double maxPivotAngle = 0.0;
        }

        public static class Positions {
            public static final double intakingPiece = 0.0; //Place holder
        }
    }

    public static class Elevator {
        public static final int leadMotorID = 10; //left
        public static final int followMotor1ID = 11; //left
        public static final int followMotor2ID = 12; //right
        public static final int followMotor3ID = 13; //right

        public static final double gearRatio = (12.0/60.0) * (36.0/64.0); //
        public static final double sprocketPitchDiameter = 1.751; //inches
        public static final double encoderToInches = gearRatio * (sprocketPitchDiameter * Math.PI) * 2; //calculates instage which is double the motion of outer stage

        public static class PIDConstants{
            public static final double kP = 2.5; //placeholder
            public static final double kI = 0.0; //placeholder
            public static final double kD = 0.0; //placeholder
            public static final double kIZone = 0.0; //placeholder
            public static final double kF = 0.025;

            public static final double kMinOutput = -0.2; //placeholder
            public static final double kMaxOutput = 0.2; //placeholder

            public static final double SmartMotionMaxVel = 350.0; //placeholder
            public static final double SmartMotionMinVel = -350.0; //placeholder
            public static final double SmartMotionMaxAcc = 500.0; //placeholder
            public static final double SmartMotionAllowedError = 0.1; //placeholder


            public static final double positionTolerance = 0.0; //placeholder

            public static final double integratorRangeMin = -.2; //placeholder (Copied from Rodrigue's Code)
            public static final double integratorRangeMax = .2;
        }

        public static final double elevatorHeightToleranceInch = 0.5;

        public static class Limits {
            public static final double softStopBottom = 3.0; //bottom soft stop (in)
            public static final double softStopTop = 40.0; //bottom soft stop (in) //PlaceHolder
            public static final double hardStopTop = 23.0; //bottom soft stop (in)

            public static final double hardStopCurrentLimit = 10.0; // Place Holder
        }

        public static class Positions {
            public static final double intakingPiece = 0.0;
        }
    }
}
