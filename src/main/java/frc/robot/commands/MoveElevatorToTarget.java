// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterPivot;

public class MoveElevatorToTarget extends Command {

  private Elevator mElevator;
  private ShooterPivot mShooterPivot;
  /** Creates a new MoveElevatorToTarget. */
  public MoveElevatorToTarget(Elevator subsystem, ShooterPivot shooterPivot) {

    mElevator = subsystem;
    mShooterPivot = shooterPivot;
    addRequirements(mElevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      mElevator.moveElevatorToTargetPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (mElevator.getTargetPosition() < Constants.Elevator.Limits.softStopBottom){
      new MoveElevator(mElevator, mShooterPivot, -0.01).schedule();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mElevator.atPosition();
  }
}
