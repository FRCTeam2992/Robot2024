// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.MoveFeeder;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.MoveShooterPivot;
import frc.robot.commands.StartShooter;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopShooter;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class RobotContainer {

  public final Shooter mShooter;
  private Intake mIntake;
  private Feeder mFeeder;
  private ShooterPivot mShooterPivot;

  public RobotContainer() {

    mShooter = new Shooter();
    mShooter.setDefaultCommand(new StopShooter(mShooter));

    mIntake = new Intake();
    mIntake.setDefaultCommand(new StopIntake(mIntake));

    mFeeder = new Feeder();
    mFeeder.setDefaultCommand(new MoveFeeder(mFeeder, 0.0));

    mShooterPivot = new ShooterPivot();
    // mShooterPivot.setDefaultCommand(getAutonomousCommand());

    configureBindings();
    configureSmartDashboard();
  }

  private void configureBindings() {}

  private void configureSmartDashboard(){
  SmartDashboard.putNumber("Set Shooter RPM", 0.0);
  SmartDashboard.putData("Start Shooter", new StartShooter(mShooter));

  SmartDashboard.putData("Intake Foward", new MoveIntake(mIntake, 0.2));
  SmartDashboard.putData("Intake Reverse", new MoveIntake(mIntake, -0.2));
  
  SmartDashboard.putData("Feeder Foward", new MoveFeeder(mFeeder, 0.2));
  SmartDashboard.putData("Feeder Reverse", new MoveFeeder(mFeeder, -0.2));

  SmartDashboard.putData("Pivot Foward", new MoveShooterPivot(mShooterPivot, 0.05));
  SmartDashboard.putData("Pivot Reverse", new MoveShooterPivot(mShooterPivot, -0.05));


  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
