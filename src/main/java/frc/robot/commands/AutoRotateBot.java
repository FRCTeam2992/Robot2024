// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class AutoRotateBot extends Command {

  private Drivetrain mDrivetrain;
  private boolean mIsRotate;
  /** Creates a new AutoRotateBot. */
  public AutoRotateBot(Drivetrain subsystem, boolean isRotate) {

    mDrivetrain = subsystem;
    mIsRotate = isRotate;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrivetrain.setAutoRotate(mIsRotate);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrivetrain.setAutoRotate(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}