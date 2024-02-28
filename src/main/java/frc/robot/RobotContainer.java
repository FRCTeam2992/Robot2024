// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.drive.swerve.OdometryThread;
import frc.robot.commands.DisableFieldOreintToggle;
import frc.robot.commands.DriveSticks;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.SetSwerveAngle;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  public final Drivetrain mDrivetrain;
  public final RobotState mRobotState;
  public OdometryThread mOdometryThread;


  public final CommandXboxController controller0 = new CommandXboxController(0);

  public RobotContainer() {

    mRobotState = new RobotState();
    
    mDrivetrain = new Drivetrain();
    if (Constants.DrivetrainConstants.odometryThread) {
      mOdometryThread = new OdometryThread(mDrivetrain);
      mOdometryThread.setFastMode();
      mOdometryThread.start();
    }
    mDrivetrain.setDefaultCommand(new DriveSticks(mDrivetrain, mRobotState));

    configureBindings();
  }

  private void configureBindings() {

    controller0.start().onTrue(new ResetGyro(mDrivetrain));

    SmartDashboard.putData(new SetSwerveAngle(mDrivetrain, 90.0, 90.0, 90.0, 90.0));

    // controller0.rightBumper().whileTrue(new DisableFieldOreintToggle(mDrivetrain));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public CommandXboxController getController0() {
    return controller0;
  }
}
