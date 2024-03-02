// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;

/*
 * This command is "set and forget" and ends immediately.  Pivot will move to target angle and hold there.
 * 
 */

public class SetPivotTargetAngle extends Command {
  /** Creates a new setPivotTargetAngle. */
  private ShooterPivot mPivot;
  private double mAngle;
  public SetPivotTargetAngle(ShooterPivot subsystem, double angle) {
    mPivot = subsystem;
    mAngle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mPivot);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mPivot.setPivotTarget(mAngle);
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
