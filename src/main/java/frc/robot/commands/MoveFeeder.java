// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

public class MoveFeeder extends Command {
  /** Creates a new moveFeeder. */
  private Feeder mFeeder;
  private double mSpeed;
  private boolean mIsBeamBreakLimited;

  public MoveFeeder(Feeder subsystem, double speed, boolean isBeamBreakLimited) {
    // Use addRequirements() here to declare subsystem dependencies.
    mFeeder = subsystem;
    mSpeed = speed;
    mIsBeamBreakLimited = isBeamBreakLimited;
    addRequirements(mFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mFeeder.setBeamBreakControl(mIsBeamBreakLimited);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mFeeder.setFeederSpeed(mSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (mIsBeamBreakLimited){
      return mFeeder.getBeamBreakTriggered();
    }
    return false;
  }
}
