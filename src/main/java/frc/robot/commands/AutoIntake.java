// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterPivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntake extends SequentialCommandGroup {
  /** Creates a new AutoIntake. */
  public AutoIntake(Elevator mElevator, Feeder mFeeder, Intake mIntake, ShooterPivot mShooterPivot) {

    addCommands(

    new SetElevatorTargetPosition(mElevator, Constants.Elevator.Positions.intakingPiece),
    new SetPivotTargetAngle(mShooterPivot, Constants.ShooterPivot.Positions.intakingPiece),
    
    new ParallelCommandGroup(
        new MoveElevatorToTarget(mElevator),
        new SetPivotToTargetAngle(mShooterPivot)
      ),

      new ParallelDeadlineGroup(
        new MoveFeeder(mFeeder, 0.2, true),
        new MoveIntake(mIntake, 0.4)
      )
      
    );
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
