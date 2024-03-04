// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ShooterPivot;

public class ElevatorSticks extends Command {

  private Elevator mElevator;
  private ShooterPivot mShooterPivot;
  /** Creates a new ClimbSticks. */
  public ElevatorSticks(Elevator elevator, ShooterPivot shooterPivot) {
    mElevator = elevator;
    mShooterPivot = shooterPivot;
    addRequirements(mElevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mElevator.setElevatorManualMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double climbY;

    climbY = Robot.mRobotContainer.controller1.getLeftY();

    if (climbY < Constants.Elevator.Climb.joyStickDeadBand){
      mElevator.setHoldPositionRecorded(false);
      mElevator.holdElevator();
    } 
      climbY = climbY*climbY*climbY;
      mElevator.setElevatorVelocity(climbY * 300);
    
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
