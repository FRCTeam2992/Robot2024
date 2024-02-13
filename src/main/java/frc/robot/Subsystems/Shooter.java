// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonFX shooterMotor;

  private TalonFXConfiguration shooterMotorConfigs;

  private DutyCycleOut percentOutControlRequest;
  private VelocityDutyCycle velocityControlRequest;
  private MotionMagicVelocityDutyCycle motionMagicVelocityControlRequest;

  private double shooterTargetRPM;

  private StatusSignal<Double> shooterVelocity;


  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor = new TalonFX(Constants.Shooter.DeviceIDs.shooterMotorID);
    shooterMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterMotorConfigs.Slot0.kP = Constants.Shooter.ShooterPIDConstants.P;
    shooterMotorConfigs.Slot0.kI = Constants.Shooter.ShooterPIDConstants.I;
    shooterMotorConfigs.Slot0.kD = Constants.Shooter.ShooterPIDConstants.D;
    shooterMotorConfigs.Slot0.kV = Constants.Shooter.ShooterPIDConstants.V;
    shooterMotorConfigs.MotionMagic.MotionMagicAcceleration = Constants.Shooter.ShooterPIDConstants.acceleration;
    shooterMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.Shooter.ShooterPIDConstants.cruiseVelocity;
    shooterMotor.getConfigurator().apply(shooterMotorConfigs);

    percentOutControlRequest = new DutyCycleOut(0.0);
    velocityControlRequest = new VelocityDutyCycle(0.0);
    motionMagicVelocityControlRequest = new MotionMagicVelocityDutyCycle(0.0);

    shooterVelocity = shooterMotor.getVelocity();

  }

  public void setShooterSpeed (double percentSpeed) {
    percentOutControlRequest.Output = percentSpeed;
    shooterMotor.setControl(percentOutControlRequest);
  }

  public void setShooterTargetRPM(double shooterSetRPM) {
    this.shooterTargetRPM = shooterSetRPM;
  }

  public void setShooterToTargetRPM () {
    double speed = shooterTargetRPM;
    setShooterRawVelocity(speed);
  }

  public void setShooterRawVelocity (double velocity) {
    velocityControlRequest.Velocity = velocity;
    shooterMotor.setControl(velocityControlRequest);
  }

  public double getShooterRPM (){
    return shooterVelocity.waitForUpdate(0.0).getValue();
  }

  public double getShooterTargetRPM () {
    return shooterTargetRPM;
  }

  public boolean atShooterRPM () {
    return (Math.abs(getShooterTargetRPM() - getShooterRPM()) < 200.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
