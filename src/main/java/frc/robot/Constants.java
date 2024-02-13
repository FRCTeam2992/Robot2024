// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */

public class Constants {

    public static class Intake{
        public static final int intakeMotorID = 0;
    }

    public static class Feeder {
        public static final int feederMotorID = 4;
    }

    public static class Shooter{
        
        public static class DeviceIDs{
            public static int shooterMotorID = 0;
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
}
