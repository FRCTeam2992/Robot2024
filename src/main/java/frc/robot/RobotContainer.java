// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveFeeder;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.MoveShooter;
import frc.robot.commands.MoveShooterPivot;
import frc.robot.commands.SetElevatorTargetPosition;
import frc.robot.commands.StartShooter;
import frc.robot.commands.StopElevator;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopShooter;
import frc.robot.commands.ZeroElevator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class RobotContainer {

  public final Shooter mShooter;
  private Intake mIntake;
  private Feeder mFeeder;
  private ShooterPivot mShooterPivot;
  public final Elevator mElevator;
  public final PowerDistribution mPDH;

  public CommandXboxController controller0;

  public RobotContainer() {

    mShooter = new Shooter();
    mShooter.setDefaultCommand(new StopShooter(mShooter));

    mIntake = new Intake();
    mIntake.setDefaultCommand(new StopIntake(mIntake));

    mFeeder = new Feeder();
    mFeeder.setDefaultCommand(new MoveFeeder(mFeeder, 0.0, false));

    mShooterPivot = new ShooterPivot();

    mElevator = new Elevator();
    mElevator.setDefaultCommand(new StopElevator(mElevator));

    mPDH = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

    controller0 = new CommandXboxController(0);
    // mShooterPivot.setDefaultCommand(getAutonomousCommand());

    configureBindings();
    configureSmartDashboard();

  }

  private void configureBindings() {
    
    controller0.povUp().whileTrue(new MoveElevator(mElevator, mShooterPivot, 0.09));
    controller0.povDown().whileTrue(new MoveElevator(mElevator, mShooterPivot, -0.05));

   }

  private void configureShuffleBoard(){
    SmartDashboard.putData("Ele 5 inch", new SetElevatorTargetPosition(mElevator, 5.0));
    SmartDashboard.putData("Ele 15 inch", new SetElevatorTargetPosition(mElevator, 15.0));

    // SmartDashboard.putNumber("Set Elevator Target Position", 0.0);
    SmartDashboard.putData(new ZeroElevator(mElevator));

  }

  private void configureSmartDashboard(){
  SmartDashboard.putNumber("Set Shooter RPM", 0.0);
  // SmartDashboard.putData("Start Shooter", new StartShooter(mShooter));

  SmartDashboard.putData("Intake Foward", new MoveIntake(mIntake, 0.40)); //2.25
  SmartDashboard.putData("Intake Reverse", new MoveIntake(mIntake, -0.40));
  
  SmartDashboard.putData("Feeder Foward", new MoveFeeder(mFeeder, 0.45, false)); //2:1
  SmartDashboard.putData("Feeder Reverse", new MoveFeeder(mFeeder, -0.45, false));
  SmartDashboard.putData("Feeder Shoot", new MoveFeeder(mFeeder, .9, false).alongWith(new MoveIntake(mIntake, 0.4)));

  SmartDashboard.putData("Pivot Foward", new MoveShooterPivot(mShooterPivot, mElevator, 0.05));
  SmartDashboard.putData("Pivot Reverse", new MoveShooterPivot(mShooterPivot, mElevator, -0.05));

  SmartDashboard.putData("Start Shooter", new MoveShooter(mShooter, -0.8));

  SmartDashboard.putData("AutoIntake", new MoveIntake(mIntake, 0.4).alongWith(new MoveFeeder(mFeeder, 0.45, false)));


  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
