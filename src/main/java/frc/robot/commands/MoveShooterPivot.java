// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterPivot;

public class MoveShooterPivot extends Command {
  /** Creates a new moveShooterPivot. */
  private ShooterPivot mPivot;
  private Elevator mElevator;
  private double mSpeed;

  public MoveShooterPivot(ShooterPivot subsystem, Elevator elevator, double speed) {
    mPivot = subsystem;
    mElevator = elevator;
    mSpeed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mElevator.getElevatorInches() < Constants.Elevator.Limits.dangerZone 
    && mPivot.getPivotAngle() < Constants.ShooterPivot.Limits.pivotCollisionZone && mSpeed < 0.0) {
      mPivot.setPivotSpeed(0.0);
    } else {
      mPivot.setPivotSpeed(mSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
