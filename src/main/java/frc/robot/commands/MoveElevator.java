// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterPivot;

public class MoveElevator extends Command {
  /** Creates a new moveElevator. */
  private Elevator mElevator;
  private ShooterPivot mShooterPivot;
  private double mSpeed;

  public MoveElevator(Elevator subsystem, ShooterPivot shooterPivot, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    mElevator = subsystem;
    mShooterPivot = shooterPivot;
    mSpeed = speed;
    addRequirements(mElevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if (mShooterPivot.getPivotAngle() < Constants.ShooterPivot.Limits.pivotCollisionZone){
    //   new SetPivotTargetAngle(mShooterPivot, Constants.ShooterPivot.Positions.pivotSafeZone).schedule();
    //   new SetPivotToTargetAngle(mShooterPivot).schedule();
    // }
  }

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
      mElevator.setElevatorSpeed(mSpeed);
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
