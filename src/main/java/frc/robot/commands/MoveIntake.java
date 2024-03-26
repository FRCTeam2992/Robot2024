// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class MoveIntake extends Command {
  /** Creates a new MoveIntake. */
  private Intake mIntake;
  private double mSpeed;
  private boolean mIsLimited;
  public MoveIntake(Intake subsystem, double speed, boolean isLimited) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIntake = subsystem;
    mSpeed = speed;
    mIsLimited = isLimited;
    addRequirements(mIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mIntake.setIntakeSpeed(mSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntake.setIntakeSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (mIsLimited){
      return mIntake.isBeamBreakLimited();
    }
    return false;
  }
}
