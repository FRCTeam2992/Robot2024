// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.html.HTMLDocument.HTMLReader.SpecialAction;
import javax.xml.stream.events.EndDocument;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.MyRobotState;
import frc.robot.MyRobotState.LEDModeState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class AutoShoot extends Command {

  private Intake mIntake;
  private Feeder mFeeder;
  private MyRobotState mState;
  private Elevator mElevator;
  private ShooterPivot mPivot;
  private Shooter mShooter;
  private double mWaitTime;

  private Timer timer;

  /** Creates a new AutoShoot. */
  public AutoShoot(Intake intake, Feeder feeder, MyRobotState state, Elevator elevator,
      ShooterPivot pivot, Shooter shooter, double waitTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIntake = intake;
    mFeeder = feeder;
    mState = state;
    mElevator = elevator;
    mPivot = pivot;
    mShooter = shooter;
    mWaitTime = waitTime;
    addRequirements(mIntake, mFeeder);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (mState.getRobotMode()) {
      case Override: {
        mIntake.setIntakeSpeed(Constants.Intake.Speeds.intakingPieceSpeed);
        mFeeder.setBeamBreakControl(false);
        mFeeder.setFeederSpeed(Constants.Feeder.Speeds.speekerShootingSpeed);
        break;

      }
      case Passing:
      case Speaker:
      case DefaultSpeaker:
      case Auto: {
        // if (timer.get() > mWaitTime ||
        //     (mElevator.atPosition() && mPivot.atTarget() && mShooter.atShooterRPM())) {
          mIntake.setIntakeSpeed(Constants.Intake.Speeds.intakingPieceSpeed);
          mFeeder.setBeamBreakControl(false);
          mFeeder.setFeederSpeed(Constants.Feeder.Speeds.speekerShootingSpeed);
        // }
        break;
      }
      case Amp: {
        mFeeder.setBeamBreakControl(false);
        mFeeder.setFeederSpeed(Constants.Feeder.Speeds.ampShootingSpeed);
        break;
      }
      case Endgame: {
        mFeeder.setBeamBreakControl(false);
        mFeeder.setFeederSpeed(Constants.Feeder.Speeds.trapShootingSpeed);
        break;
      }
    }

    mState.setLEDMode(LEDModeState.shooting);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mState.setLEDMode(LEDModeState.idle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
