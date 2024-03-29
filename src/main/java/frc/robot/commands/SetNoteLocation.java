// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MyRobotState;
import frc.robot.MyRobotState.RobotNoteLocation;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

public class SetNoteLocation extends Command {
  private Feeder mFeeder;
  private Intake mIntake;
  private MyRobotState mRobotState;

  /** Creates a new SetNoteLocation. */
  public SetNoteLocation(Feeder feeder, Intake intake, MyRobotState robotState) {
    mFeeder = feeder;
    mIntake = intake;
    mRobotState = robotState;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(mIntake.isBeamBreakLimited()){
      mRobotState.setNoteLocation(RobotNoteLocation.intake);
    }else if (mFeeder.getBeamBreakTriggered()) {
      mRobotState.setNoteLocation(RobotNoteLocation.shooter);
    } else {
      mRobotState.setNoteLocation(RobotNoteLocation.noNote);
    }

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
