// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterPivot;

public class HoldShooterPivot extends Command {

  private ShooterPivot mShooterPivot;
  private Elevator mElevator;
  /** Creates a new HoldShooterPivot. */
  public HoldShooterPivot(ShooterPivot shooterPivot, Elevator elevator) {
    mShooterPivot = shooterPivot;
    mElevator = elevator;
    addRequirements(mShooterPivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mElevator.isAboveHeightLimit(mShooterPivot.getEncoderAngle())){
      mShooterPivot.setHoldPosition((Math.sinh((Constants.ShooterPivot.robotMaxHeight - 
      Constants.ShooterPivot.pivotHeight-mElevator.getElevatorInches() - Constants.ShooterPivot.flyWheelRadius)/Constants.ShooterPivot.shooterLength)) - 19.537819);
    } else if (mElevator.getElevatorInches() < Constants.Elevator.Limits.intakeDangerZone){
      mShooterPivot.setHoldPosition(Constants.ShooterPivot.Positions.pivotSafeZone);
    }
    mShooterPivot.holdPivot();
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
