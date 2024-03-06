// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class AutoMoveForwardBack extends Command {

  private Drivetrain mDrivetrain;
  private boolean mIsFoward;
  private double mVelocity;
  /** Creates a new AutoMoveForwardBack. */
  public AutoMoveForwardBack(Drivetrain drivetrain, boolean foward, double velocity) {

    mDrivetrain = drivetrain;
    mIsFoward = foward;
    mVelocity = velocity;

    addRequirements(mDrivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrivetrain.moveRobotFrontBack(mIsFoward, mVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrivetrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
