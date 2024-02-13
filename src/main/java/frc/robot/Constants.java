package frc.robot;

public class Constants {
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

        public static final double elevatorHeightToleranceInch = 1.75;

        public static class Limits {
            public static final double softStopBottom = 3.0; //bottom soft stop (in)
            public static final double softStopTop = 40.0; //bottom soft stop (in) //PlaceHolder
            public static final double hardStopTop = 23.0; //bottom soft stop (in)

            public static final double hardStopCurrentLimit = 10.0; // Place Holder
        }
    }
}
