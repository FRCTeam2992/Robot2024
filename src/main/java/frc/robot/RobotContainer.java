// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.HoldElevatorPosition;
import frc.robot.commands.StopElevator;
import frc.robot.commands.ZeroElevator;
import frc.robot.commands.moveElevator;
import frc.robot.commands.setElevatorPosition;
import frc.robot.subsystems.Elevator;

public class RobotContainer {

  public final Elevator mElevator;

  public CommandXboxController controller0;


  public RobotContainer() {

    mElevator = new Elevator();
    mElevator.setDefaultCommand(new StopElevator(mElevator));

    controller0 = new CommandXboxController(0);


    configureBindings();
    configureShuffleBoard();
  }

  private void configureBindings() {
    
    controller0.povUp().whileTrue(new moveElevator(mElevator, 0.2));
    controller0.povDown().whileTrue(new moveElevator(mElevator, -0.2));

   }

  private void configureShuffleBoard(){
    SmartDashboard.putData(new setElevatorPosition(mElevator, SmartDashboard.getNumber("Elevator Target Postion", 10)));
    SmartDashboard.putData(new ZeroElevator(mElevator));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
