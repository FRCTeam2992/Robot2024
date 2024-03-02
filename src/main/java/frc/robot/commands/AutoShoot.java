// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.MyRobotState;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends SequentialCommandGroup {
  /** Creates a new AutoShoot. */
  public AutoShoot(Elevator mElevator, Feeder mFeeder, Intake mIntake, ShooterPivot mShooterPivot, Shooter mShooter, MyRobotState mMyRobotState) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    if (mMyRobotState.isSpeakerMode()){ //TODO check if in Speaker Robot state
      addCommands(
        // new IsReadyToShoot(mElevator, mShooterPivot, mShooter), 

        new ParallelDeadlineGroup(
          new MoveFeeder(mFeeder, Constants.Feeder.Speeds.speekerShootingSpeed, true),
          new MoveIntake(mIntake, Constants.Intake.Speeds.intakingPieceSpeed)
        )
      );
    }

     if (true){ //TODO check if in Amp Robot state
      addCommands(
        new IsReadyToShoot(mElevator, mShooterPivot, mShooter), 
        new MoveFeeder(mFeeder, Constants.Feeder.Speeds.ampShootingSpeed, true)
      );
    }

    if (true){ //TODO check if in Trap Robot state
      addCommands(
        new IsReadyToShoot(mElevator, mShooterPivot, mShooter),
        new MoveFeeder(mFeeder, Constants.Feeder.Speeds.trapShootingSpeed, true)
      );
    }


  }
}
