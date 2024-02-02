// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public static class ShooterPivot {
        public static final int pivotMotorID = 3;

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
    }
}
