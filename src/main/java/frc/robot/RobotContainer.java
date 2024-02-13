// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.StopElevator;
import frc.robot.commands.ZeroElevator;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.subsystems.Elevator;

public class RobotContainer {

  public final Elevator mElevator;
  public final PowerDistribution mPDH;

  public CommandXboxController controller0;


  public RobotContainer() {

    mElevator = new Elevator();
    mElevator.setDefaultCommand(new StopElevator(mElevator));

    mPDH = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

    controller0 = new CommandXboxController(0);


    configureBindings();
    configureShuffleBoard();

    SmartDashboard.putData(mElevator);
  }

  private void configureBindings() {
    
    controller0.povUp().whileTrue(new MoveElevator(mElevator, 0.09));
    controller0.povDown().whileTrue(new MoveElevator(mElevator, -0.05));

   }

  private void configureShuffleBoard(){
    SmartDashboard.putData("Ele 5 inch", new SetElevatorPosition(mElevator, 5.0));
    SmartDashboard.putData("Ele 15 inch", new SetElevatorPosition(mElevator, 15.0));

    // SmartDashboard.putNumber("Set Elevator Target Position", 0.0);
    SmartDashboard.putData(new ZeroElevator(mElevator));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
