// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.MyRobotState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.lib.range.NoteInterpolator;

public class AutoAim extends Command {

  private Elevator mElevator;
  private ShooterPivot mShooterPivot;
  private Shooter mShooter;
  private MyRobotState mState;
  private NoteInterpolator mInterpolator;
  private Drivetrain mDrivetrain;

  private boolean abort = false;

  /** Creates a new AutoAim. */
  public AutoAim(Elevator elevator, ShooterPivot pivot, Shooter shooter, MyRobotState state,
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
    switch (mState.getRobotMode()) {
      case Override: {
        abort = true;
        break;
      }
      case Speaker:
      case Auto: {
        double distance = calculateDistance();

        mShooter.setShooterTargetRPM(mInterpolator.calcMainShooterSpeed(distance));
        mShooter.setShooterToTargetRPM();
        mShooterPivot.setPivotTarget(mInterpolator.calcPivotPosition(distance));
        mShooterPivot.setPivotToPID();
        mElevator.setElevatorTargetPosition(Constants.Elevator.Positions.speakerShooting); // All the way down
        mElevator.moveElevatorToTargetPosition();
        break;
      }
      case DefaultSpeaker: {
        mShooter.setShooterTargetRPM(3000);
        mShooter.setShooterToTargetRPM();
        mShooterPivot.setPivotTarget(47.5);
        mShooterPivot.setPivotToPID();
        mElevator.setElevatorTargetPosition(Constants.Elevator.Positions.speakerShooting); // All the way down
        mElevator.moveElevatorToTargetPosition();
        break;
      }

      case Amp: {
        mShooter.setShooterSpeed(0.0);
        mShooterPivot.setPivotTarget(0.0);
        mShooterPivot.setPivotToPID();
        mElevator.setElevatorTargetPosition(Constants.Elevator.Positions.ampScoring);
        mElevator.moveElevatorToTargetPosition();
        break;
      }

      case Endgame: {
        mShooter.setShooterSpeed(0.0);
        mShooterPivot.setPivotTarget(0.0);
        mShooterPivot.setPivotToPID();
        mElevator.setElevatorTargetPosition(Constants.Elevator.Positions.trapScoring);
        mElevator.moveElevatorToTargetPosition();
        break;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    switch (mState.getRobotMode()) {
      case Override: {
        // Do nothing
      }

      case Auto:
      case Speaker:
      case DefaultSpeaker: {
        mShooterPivot.setPivotTarget(Constants.ShooterPivot.Positions.intakingPiece);
        mShooterPivot.setPivotToPID();
        mElevator.setElevatorTargetPosition(Constants.Elevator.Positions.intakingPiece);
        mElevator.moveElevatorToTargetPosition();
        mShooter.setShooterTargetRPM(Constants.Shooter.defaultShooterSpeed);
        mShooter.setShooterToTargetRPM();
        break;
      }

      case Amp:
      case Endgame: {
        mShooterPivot.setPivotTarget(Constants.ShooterPivot.Positions.intakingPiece);
        mShooterPivot.setPivotToPID();
        mElevator.setElevatorTargetPosition(Constants.Elevator.Positions.intakingPiece);
        mElevator.moveElevatorToTargetPosition();
        mShooter.setShooterSpeed(0.0);
        break;
      }

    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return abort;
  }

  private double calculateDistance() {
    // goal changes depending on alliance
    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == (DriverStation.Alliance.Red)) {
      return mDrivetrain.latestSwervePose.getTranslation()
          .getDistance(Constants.DrivetrainConstants.Field.redGoalTarget);
    } else {
      return mDrivetrain.latestSwervePose.getTranslation()
          .getDistance(Constants.DrivetrainConstants.Field.blueGoalTarget);
    }
  }
}
