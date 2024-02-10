// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.HoldElevatorPosition;
import frc.robot.commands.StopElevator;
import frc.robot.commands.setElevatorPosition;
import frc.robot.subsystems.Elevator;

public class RobotContainer {

  public final Elevator mElevator;

  public RobotContainer() {

    mElevator = new Elevator();
    mElevator.setDefaultCommand(new StopElevator(mElevator));

    configureBindings();
    configureShuffleBoard();
  }

  private void configureBindings() { }

  private void configureShuffleBoard(){
    SmartDashboard.putData(new setElevatorPosition(mElevator, SmartDashboard.getNumber("Elevator Target Postion", 10)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
