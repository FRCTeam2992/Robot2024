// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.range.NoteInterpolator;
import frc.robot.Constants;
import frc.robot.MyRobotState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class AutonomousLimitedAim extends Command {

  private Elevator mElevator;
  private ShooterPivot mShooterPivot;
  private Shooter mShooter;
  private MyRobotState mState;
  private NoteInterpolator mInterpolator;
  private Drivetrain mDrivetrain;

  private boolean abort = false;

  /** Creates a new AutonomousLimitedAim. */
  public AutonomousLimitedAim(Elevator elevator, ShooterPivot pivot, Shooter shooter, MyRobotState state,
  NoteInterpolator interp, Drivetrain drivetrain) {
// Use addRequirements() here to declare subsystem dependencies.
mElevator = elevator;
mShooterPivot = pivot;
mShooter = shooter;
mState = state;
mInterpolator = interp;
mDrivetrain = drivetrain;
addRequirements(elevator, pivot, shooter);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    abort = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = calculateDistance();
    Math.max(distance, Constants.ShooterPivot.Limits.minAimDistanceClearOfStage);
        SmartDashboard.putNumber("Aim Distance", distance);

        mDrivetrain.setAutoRotate(true);
        mShooter.setShooterTargetRPM(mInterpolator.calcMainShooterSpeed(distance));
        mShooter.setShooterToTargetRPM();
        mShooterPivot.setPivotTarget(mInterpolator.calcPivotPosition(distance));
        mShooterPivot.setPivotToPID();
        mElevator.setElevatorTargetPosition(mInterpolator.calcElevatorHeight(distance)); // All the way down
        mElevator.moveElevatorToTargetPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

   private double calculateDistance() {
    // goal changes depending on alliance
    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == (DriverStation.Alliance.Red)) {
      return mDrivetrain.latestSwervePose.getTranslation()
          .getDistance(Constants.DrivetrainConstants.Field.redGoalTarget) * 39.3701; // Inches
    } else {
      return mDrivetrain.latestSwervePose.getTranslation()
          .getDistance(Constants.DrivetrainConstants.Field.blueGoalTarget) * 39.3701; // Inches
    }
  }
}
