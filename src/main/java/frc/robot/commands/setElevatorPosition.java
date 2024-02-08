// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Constants;
import frc.robot.subsystems.Elevator;

public class setElevatorPosition extends Command {
  /** Creates a new setElevatorPosition. */
  private Elevator mElevator;
  private double mPosition;

  public setElevatorPosition(Elevator subsystem, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    mElevator = subsystem;
    mPosition = position;
    addRequirements(mElevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mElevator.setElevatorPosition(mPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (mPosition < Constants.Elevator.Limits.softStopBottom){
      new moveElevator(mElevator, -0.05);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mElevator.atPosition();
  }
}
