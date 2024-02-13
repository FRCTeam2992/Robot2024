// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  /** Creates a new feeder. */
  private TalonFX feedMotor;
  private TalonFXConfiguration feedMotorConfigs;
  private DutyCycleOut percentOutControlRequest;

  private boolean isLimited = false; //in preparation for limit sensor

  public Feeder() {
    feedMotor = new TalonFX(Constants.Feeder.feederMotorID);
    feedMotorConfigs = new TalonFXConfiguration();

    feedMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    feedMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    feedMotor.getConfigurator().apply(feedMotorConfigs);

    percentOutControlRequest = new DutyCycleOut(0.0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putData(this);
    // This method will be called once per scheduler run
  }

  public void setFeederSpeed(double speed) {
    if (!isLimited) {
      feedMotor.setControl(percentOutControlRequest.withOutput(speed));
    }
  }
}
