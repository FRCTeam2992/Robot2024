// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterPivot;

public class SetElevatorVelocity extends Command {

  private Elevator mElevator;
  private ShooterPivot mShooterPivot;
  private double mVelocity;
  /** Creates a new SetElevatorVelocity. */
  public SetElevatorVelocity(Elevator elevator, ShooterPivot shooterPivot, double velocity) {

    mElevator = elevator;
    mShooterPivot = shooterPivot;
    mVelocity = velocity;

    addRequirements(mElevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mShooterPivot.getPivotAngle() < Constants.ShooterPivot.Limits.pivotCollisionZone 
    && mElevator.getElevatorInches() < Constants.Elevator.Limits.elevatorDangerZone) {
      mElevator.setHoldPositionRecorded(false);
      mElevator.holdElevator();
      new SetPivotTargetAngle(mShooterPivot, Constants.ShooterPivot.Positions.pivotSafeZone).schedule();
      new SetPivotToTargetAngle(mShooterPivot).schedule();
    } else { 
      mElevator.setElevatorSpeed(mVelocity);
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
