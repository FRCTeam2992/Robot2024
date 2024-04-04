// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MyRobotState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class SetOnTarget extends Command {

  private Elevator mElevator;
  private Shooter mShooter;
  private ShooterPivot mPivot;
  private MyRobotState mRobotState;

  /** Creates a new SetOnTarget. */
  public SetOnTarget(Elevator elevator, Shooter shooter, ShooterPivot pivot, MyRobotState robotState) {
    mElevator = elevator;
    mShooter = shooter;
    mPivot = pivot;
    mRobotState = robotState;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mRobotState.setOnTarget((mElevator.atPosition() && mPivot.atTarget() && mShooter.atShooterRPM()));
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
