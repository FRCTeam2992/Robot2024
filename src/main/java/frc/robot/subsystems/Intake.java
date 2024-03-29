// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFX intakeLeadMotor;
  // private TalonFX intakeFollowMotor;

  private DigitalInput beamBreak;

  private TalonFXConfiguration intakeMotorConfigs;
  private DutyCycleOut percentOutControlRequest;

  public Intake() {
    intakeLeadMotor = new TalonFX(Constants.Intake.intakeLeadMotorID, "CanBus2");
    // intakeFollowMotor = new TalonFX(Constants.Intake.intakeFollowMotorID, "CanBus2");
    intakeMotorConfigs = new TalonFXConfiguration();

    intakeMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    intakeLeadMotor.getConfigurator().apply(intakeMotorConfigs);
    // intakeFollowMotor.getConfigurator().apply(intakeMotorConfigs);

    // intakeFollowMotor.setControl(new Follower(Constants.Intake.intakeLeadMotorID,
        // false));

    beamBreak = new DigitalInput(Constants.Intake.intakeBeamBreakID);

    percentOutControlRequest = new DutyCycleOut(0.0).withEnableFOC(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(this);
    // This method will be called once per scheduler run
  }

  public void setIntakeSpeed(double speed) {
    intakeLeadMotor.setControl(percentOutControlRequest.withOutput(speed));
    // intakeFollowMotor.setControl(percentOutControlRequest.withOutput(speed));

  }

  public void resetSubsystemState() {
    setIntakeSpeed(0.0);
  }

  public boolean isBeamBreakLimited() {
    return beamBreak.get();
  }
}
