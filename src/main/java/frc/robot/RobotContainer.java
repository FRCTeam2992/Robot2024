// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.MoveFeeder;
import frc.robot.Commands.StopIntake;
import frc.robot.Commands.StopShooter;
import frc.robot.Subsystems.Feeder;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class RobotContainer {

  public final Shooter mShooter;
  private Intake mIntake;
  private Feeder mFeeder;

  public RobotContainer() {

    mShooter = new Shooter();
    mShooter.setDefaultCommand(new StopShooter(mShooter));

    mIntake = new Intake();
    mIntake.setDefaultCommand(new StopIntake(mIntake));

    mFeeder = new Feeder();
    mFeeder.setDefaultCommand(new MoveFeeder(mFeeder, 0.0));

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
