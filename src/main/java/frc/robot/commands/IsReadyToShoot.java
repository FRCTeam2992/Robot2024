// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class IsReadyToShoot extends Command {

  private Elevator mElevator;
  private ShooterPivot mShooterPivot;
  private Shooter mShooter;
  // TODO add drivetrain rotation check

  /** Creates a new IsReadyToShoot. */
  public IsReadyToShoot(Elevator elevator, ShooterPivot shooterPivot, Shooter shooter) {

    mElevator = elevator;
    mShooterPivot = shooterPivot;
    mShooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (mElevator.atPosition() && mShooterPivot.atTarget() && mShooter.atShooterRPM());
  }
}
