// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFX intakeMotor;
  private TalonFXConfiguration intakeMotorConfigs;
  private DutyCycleOut percentOutControlRequest;
  public Intake() {
    intakeMotor = new TalonFX(Constants.Intake.intakeMotorID);
    intakeMotorConfigs = new TalonFXConfiguration();

    intakeMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    intakeMotor.getConfigurator().apply(intakeMotorConfigs);

    percentOutControlRequest = new DutyCycleOut(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(this);
    // This method will be called once per scheduler run
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.setControl(percentOutControlRequest.withOutput(speed));
  }
}
