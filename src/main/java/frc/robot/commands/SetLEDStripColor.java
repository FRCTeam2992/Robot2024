// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.leds.Color;
import frc.robot.subsystems.LEDs;

public class SetLEDStripColor extends Command {
  /** Creates a new SetLEDColor. */
  private LEDs mLED;
  private Color mColor;

  public SetLEDStripColor(LEDs LED, Color color) {
    // Use addRequirements() here to declare subsystem dependencies.
    mLED = LED;
    mColor = color;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mLED.setLEDStripColor(mColor);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
