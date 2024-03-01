// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.vision.LimeLight;
import frc.lib.vision.LimeLight.CameraMode;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetLimeLightOdometryUpdates extends InstantCommand {
    Drivetrain mDrivetrain;
    boolean mUseLimelight;

    public SetLimeLightOdometryUpdates(Drivetrain drivetrain, boolean useLimelight) {
        mDrivetrain = drivetrain;
        mUseLimelight = useLimelight;
        runsWhenDisabled();
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mDrivetrain.setLimeLightOdometryUpdates(mUseLimelight);
        for (LimeLight camera : mDrivetrain.limelightList) {
            if (mUseLimelight) {
                camera.setCameraMode(CameraMode.Vision);
            } else {
                camera.setCameraMode(CameraMode.Driver);
            }
        }
    }
}
