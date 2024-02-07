// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DisableFieldOreintToggle;
import frc.robot.Commands.DriveSticks;
import frc.robot.Commands.ResetGyro;
import frc.robot.Subsystems.Drivetrain;

public class RobotContainer {

  public final Drivetrain mDrivetrain;


  public final CommandXboxController controller0 = new CommandXboxController(0);

  public RobotContainer() {
    
    mDrivetrain = new Drivetrain();
    mDrivetrain.setDefaultCommand(new DriveSticks(mDrivetrain));

    configureBindings();
  }

  private void configureBindings() {

    controller0.start().onTrue(new ResetGyro(mDrivetrain));

    // controller0.rightBumper().whileTrue(new DisableFieldOreintToggle(mDrivetrain));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public CommandXboxController getController0() {
    return controller0;
  }
}
