// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  /** Creates a new feeder. */
  private TalonFX feedMotor;
  private TalonFXConfiguration feedMotorConfigs;
  private DutyCycleOut percentOutControlRequest;

  public Feeder() {
    feedMotor = new TalonFX(Constants.Feeder.feederMotorID);
    feedMotorConfigs = new TalonFXConfiguration();

    feedMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    feedMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feedMotorConfigs.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin; // Not sure if
                                                                                                      // sensor settings
                                                                                                      // are correct
    feedMotorConfigs.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    feedMotorConfigs.HardwareLimitSwitch.ReverseLimitEnable = true;

    feedMotor.getConfigurator().apply(feedMotorConfigs);

    percentOutControlRequest = new DutyCycleOut(0.0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Is Limited", getBeamBreakTriggered());
    SmartDashboard.putData(this);
    // This method will be called once per scheduler run
  }

  public void setFeederSpeed(double speed) {
    feedMotor.setControl(percentOutControlRequest.withOutput(-speed));
  }

  public boolean getBeamBreakTriggered() {
    switch (feedMotor.getReverseLimit().refresh().getValue()) {
      case ClosedToGround:
        return true;
      case Open:
        return false;
      default:
        return true;
    }

  }

  public void setBeamBreakControl(boolean isBeamEnabled) {
    feedMotorConfigs.HardwareLimitSwitch.ReverseLimitEnable = isBeamEnabled;
    feedMotor.getConfigurator().apply(feedMotorConfigs);
  }

  public void resetSubsystemState() {
    setFeederSpeed(0.0);
  }
}
