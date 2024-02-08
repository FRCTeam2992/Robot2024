package frc;

public class Constants {
    public static class Elevator {
        public static final int leadMotorID = 11; //left
        public static final int followMotor1ID = 10; //left
        public static final int followMotor2ID = 13; //right
        public static final int followMotor3ID = 12; //right

        public static final double gearRatio = 0.0; //placeholder
        public static final double sprocketPitchDiameter = 0.0; //placeholder
        public static final double encoderElevatorToInches = gearRatio / (sprocketPitchDiameter * Math.PI);

        public static final double PIDControllerP = 0.0; //placeholder
        public static final double PIDControllerI = 0.0; //placeholder
        public static final double PIDControllerD = 0.0; //placeholder

        public static final double positionTolerance = 0.0; //placeholder

        public static final double integratorRangeMin = -.2; //placeholder (Copied from Rodrigue's Code)
        public static final double integratorRangeMax = .2;

        public static final double elevatorHeightToleranceInch = 1.75;

        public static class Limits {
            public static final double softStopBottom = 3.0; //bottom soft stop (in)
            public static final double softStopTop = 20.0; //bottom soft stop (in) //PlaceHolder
            public static final double hardStopTop = 23.0; //bottom soft stop (in)

            public static final double hardStopCurrentLimit = 10.0; // Place Holder
        }
    }
}
