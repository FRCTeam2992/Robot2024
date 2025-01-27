// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.MyRobotState;
import frc.robot.MyRobotState.LEDModeState;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntake extends SequentialCommandGroup {
  /** Creates a new AutoIntake. */
  public AutoIntake(Feeder mFeeder, Intake mIntake, MyRobotState mRobotState) {
    addCommands(
        new InstantCommand(() -> {if (mRobotState.getLEDMode() == LEDModeState.idle){mRobotState.setLEDMode(LEDModeState.intaking);}}),
        new ParallelRaceGroup(
            new MoveFeeder(mFeeder, Constants.Feeder.Speeds.intakingPieceSpeed, true),
            new MoveIntake(mIntake,
                Constants.Intake.Speeds.intakingPieceSpeed, false)
        ),
        new InstantCommand(() -> {if (mRobotState.getLEDMode() == LEDModeState.idle){mRobotState.setLEDMode(LEDModeState.idle);}}),

        new ParallelRaceGroup(
            new MoveFeeder(mFeeder, 0.15, false).withTimeout(0.35),
            new MoveIntake(mIntake, 0.05, false)
        )
        
    );

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
  }
}
