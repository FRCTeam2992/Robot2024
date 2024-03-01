// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SetLimeLightOdometryUpdates;

public class Robot extends TimedRobot {
  
  private Command m_autonomousCommand;

  public static RobotContainer mRobotContainer;

  @Override
  public void robotInit() {
    mRobotContainer = new RobotContainer();

    mRobotContainer.mDrivetrain.navx.zeroYaw();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    mRobotContainer.mDrivetrain.onDisable();

    CommandScheduler.getInstance().schedule(
          new SetLimeLightOdometryUpdates(mRobotContainer.mDrivetrain, false));

  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = mRobotContainer.getAutonomousCommand();

    mRobotContainer.mDrivetrain.setDriveNeutralMode(NeutralModeValue.Brake);
        mRobotContainer.mDrivetrain.setTurnNeutralMode(NeutralModeValue.Brake);

        // Set the Drive Motors Current Limit
    mRobotContainer.mDrivetrain.setDriveCurrentLimit(60.0, 60.0);

        // Zero the gyro
    mRobotContainer.mDrivetrain.navx.zeroYaw();

        // Set the Drive Motors Ramp Rate
    mRobotContainer.mDrivetrain.setDriveRampRate(0.0);

    CommandScheduler.getInstance().schedule(
          new SetLimeLightOdometryUpdates(mRobotContainer.mDrivetrain, true));

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

    mRobotContainer.mDrivetrain.setDriveNeutralMode(NeutralModeValue.Brake);
        mRobotContainer.mDrivetrain.setTurnNeutralMode(NeutralModeValue.Brake);

    mRobotContainer.mDrivetrain.setDriveCurrentLimit(40.0, 40.0);
        mRobotContainer.mDrivetrain.setDriveRampRate(0.25);

    CommandScheduler.getInstance().schedule(
              new SetLimeLightOdometryUpdates(mRobotContainer.mDrivetrain, true));

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
