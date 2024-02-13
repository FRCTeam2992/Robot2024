// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;

public class setPivotToTargetAngle extends Command {
  /** Creates a new setPivotToTargetAngle. */
  private ShooterPivot mPivot;
  public setPivotToTargetAngle(ShooterPivot subsystem) {
    mPivot = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mPivot.setPivotToTarget();
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
