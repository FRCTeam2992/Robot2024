// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonFX rightShooterMotor;
  private TalonFX leftShooterMotor;

  private TalonFXConfiguration rightShooterMotorConfigs;
  private TalonFXConfiguration leftShooterMotorConfigs;

  private DutyCycleOut percentOutControlRequest;
  private VelocityDutyCycle velocityControlRequest;
  private MotionMagicVelocityDutyCycle motionMagicVelocityControlRequest;

  private double rightShooterTargetRPM;
  private double leftShooterTargetRPM;

  private StatusSignal<Double> rightShooterVelocity;
  private StatusSignal<Double> leftShooterVelocity;



  /** Creates a new Shooter. */
  public Shooter() {
    rightShooterMotor = new TalonFX(Constants.Shooter.DeviceIDs.rightShooterMotorID);
    rightShooterMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightShooterMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightShooterMotorConfigs.Slot0.kP = Constants.Shooter.RightShooterPIDConstants.P;
    rightShooterMotorConfigs.Slot0.kI = Constants.Shooter.RightShooterPIDConstants.I;
    rightShooterMotorConfigs.Slot0.kD = Constants.Shooter.RightShooterPIDConstants.D;
    rightShooterMotorConfigs.Slot0.kV = Constants.Shooter.RightShooterPIDConstants.V;
    rightShooterMotorConfigs.MotionMagic.MotionMagicAcceleration = Constants.Shooter.RightShooterPIDConstants.acceleration;
    rightShooterMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.Shooter.RightShooterPIDConstants.cruiseVelocity;
    rightShooterMotor.getConfigurator().apply(rightShooterMotorConfigs);

    
    leftShooterMotor = new TalonFX(Constants.Shooter.DeviceIDs.leftShooterMotorID);
    leftShooterMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftShooterMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leftShooterMotorConfigs.Slot0.kP = Constants.Shooter.LeftShooterPIDConstants.P;
    leftShooterMotorConfigs.Slot0.kI = Constants.Shooter.LeftShooterPIDConstants.I;
    leftShooterMotorConfigs.Slot0.kD = Constants.Shooter.LeftShooterPIDConstants.D;
    leftShooterMotorConfigs.Slot0.kV = Constants.Shooter.LeftShooterPIDConstants.V;
    leftShooterMotorConfigs.MotionMagic.MotionMagicAcceleration = Constants.Shooter.LeftShooterPIDConstants.acceleration;
    leftShooterMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.Shooter.LeftShooterPIDConstants.cruiseVelocity;
    leftShooterMotor.getConfigurator().apply(leftShooterMotorConfigs);

    percentOutControlRequest = new DutyCycleOut(0.0);
    velocityControlRequest = new VelocityDutyCycle(0.0);
    motionMagicVelocityControlRequest = new MotionMagicVelocityDutyCycle(0.0);

    rightShooterVelocity = rightShooterMotor.getVelocity();

  }

  public void setRightShooterSpeed (double percentSpeed) {
    rightShooterMotor.setControl(percentOutControlRequest.withOutput(percentSpeed));
  }

  public void setLeftShooterSpeed (double percentSpeed) {
    leftShooterMotor.setControl(percentOutControlRequest.withOutput(percentSpeed));
  }

  public void setRightShooterTargetRPM(double rightShooterSetRPM) {
    this.rightShooterTargetRPM = rightShooterSetRPM;
  }

  public void setLeftShooterTargetRPM(double leftShooterSetRPM) {
    this.leftShooterTargetRPM = leftShooterSetRPM;
  }

  public void setRightShooterToTargetRPM () {
    double speed = rightShooterTargetRPM;
    setRightShooterRawVelocity(speed);
  }

  public void setLeftShooterToTargetRPM () {
    double speed = leftShooterTargetRPM;
    setLeftShooterRawVelocity(speed);
  }

  public void setRightShooterRawVelocity (double velocityControlRequest) {
    rightShooterMotor.setControl(motionMagicVelocityControlRequest.withVelocity(velocityControlRequest));
  }

  public void setLeftShooterRawVelocity (double velocityControlRequest) {
    leftShooterMotor.setControl(motionMagicVelocityControlRequest.withVelocity(velocityControlRequest));
  }

  public double getRightShooterRPM (){
    return rightShooterVelocity.waitForUpdate(0.1).getValue();
  }

  public double getLeftShooterRPM (){
    return leftShooterVelocity.waitForUpdate(0.1).getValue();
  }

  public double getRightShooterTargetRPM () {
    return rightShooterTargetRPM;
  }

  public double getLeftShooterTargetRPM () {
    return leftShooterTargetRPM;
  }

  public boolean atRightShooterRPM () {
    return (Math.abs(getRightShooterTargetRPM() - getRightShooterRPM()) < 200.0);
  }

  public boolean atLeftShooterRPM () {
    return (Math.abs(getLeftShooterTargetRPM() - getLeftShooterRPM()) < 200.0);
  }

  public boolean atShooterRPM () {
    return (atRightShooterRPM() && atLeftShooterRPM());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
