// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.commands.SetLimeLightOdometryUpdates;
import frc.robot.commands.AutoMoveForwardBack;
import frc.robot.commands.SetElevatorTargetPosition;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static RobotContainer mRobotContainer;

  @Override
  public void robotInit() {
    mRobotContainer = new RobotContainer();

    mRobotContainer.mDrivetrain.resetGyro();

    mRobotContainer.mElevator.zeroElevatorEncoders();

    mRobotContainer.mShooterPivot.zeroPivotEncoder();

    // CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // SmartDashboard.putNumber("Elevator Current Out",
    // mRobotContainer.mPDH.getCurrent(6));
    SmartDashboard.putString("Robot State", mRobotContainer.mRobotState.getRobotMode().toString());
    SmartDashboard.putBoolean("Speaker", mRobotContainer.mRobotState.isSpeakerMode());
    SmartDashboard.putBoolean("Amp", mRobotContainer.mRobotState.isAmpMode());
    SmartDashboard.putBoolean("Overried", mRobotContainer.mRobotState.IsOverrideMode());
    SmartDashboard.putBoolean("Default", mRobotContainer.mRobotState.isDefaultSpeakerMode());

  }

  @Override
  public void disabledInit() {
    mRobotContainer.mDrivetrain.onDisable();

    CommandScheduler.getInstance().schedule(
          new SetLimeLightOdometryUpdates(mRobotContainer.mDrivetrain, true));

    mRobotContainer.mRobotState.setRobotMode(MyRobotState.RobotModeState.Speaker);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    mRobotContainer.mRobotState.setRobotMode(MyRobotState.RobotModeState.Auto);
  
    m_autonomousCommand = mRobotContainer.getAutonomousCommand();

    mRobotContainer.mDrivetrain.setDriveNeutralMode(NeutralModeValue.Brake);
        mRobotContainer.mDrivetrain.setTurnNeutralMode(NeutralModeValue.Brake);

        // Set the Drive Motors Current Limit
    mRobotContainer.mDrivetrain.setDriveCurrentLimit(60.0, 60.0);

        // Zero the gyro
        mRobotContainer.mDrivetrain.resetGyro();

        // Reset encoders
        mRobotContainer.mShooterPivot.zeroPivotEncoder();
        mRobotContainer.mElevator.zeroElevatorEncoders();

        // Set the Drive Motors Ramp Rate
    mRobotContainer.mDrivetrain.setDriveRampRate(0.0);

    CommandScheduler.getInstance().schedule(
          new SetLimeLightOdometryUpdates(mRobotContainer.mDrivetrain, true));

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    // new AutoMoveForwardBack(mRobotContainer.mDrivetrain, true, .15).schedule();
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
    mRobotContainer.mRobotState.setRobotMode(MyRobotState.RobotModeState.Speaker);

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
    mRobotContainer.mRobotState.setRobotMode(MyRobotState.RobotModeState.Speaker);
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
