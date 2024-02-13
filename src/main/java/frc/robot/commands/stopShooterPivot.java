// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ShooterPivot;

public class stopShooterPivot extends Command {
  /** Creates a new stopShooterPivot. */
  private ShooterPivot mPivot;
  public stopShooterPivot(ShooterPivot subsystem) {
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
    mPivot.setPivotSpeed(0);
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
