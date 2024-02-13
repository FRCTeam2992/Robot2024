// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterPivot extends SubsystemBase {

  private TalonFX pivotMotor;
  private TalonFXConfiguration pivotMotorConfigs;

  private DutyCycleOut percentOutControlRequest;

  private PIDController pivotController;
  
  private DutyCycleEncoder lampreyEncoder;

  private double pivotTarget;

  public MedianFilter pivotMedianFilter;

  /** Creates a new ShooterPivot. */
  public ShooterPivot() {

    pivotMotor = new TalonFX(Constants.ShooterPivot.pivotMotorID, "CanBus2");
    pivotMotorConfigs = new TalonFXConfiguration();
    pivotMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotController = new PIDController(Constants.ShooterPivot.PIDController.P, 
    Constants.ShooterPivot.PIDController.I, Constants.ShooterPivot.PIDController.D, 0.02);
    pivotController.setTolerance(Constants.ShooterPivot.PIDController.pivotAngleTolerance);

    lampreyEncoder = new DutyCycleEncoder(0);
    lampreyEncoder.setDistancePerRotation(1.0);

    percentOutControlRequest = new DutyCycleOut(0.0);

    pivotMedianFilter = new MedianFilter(5);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(this);
    // This method will be called once per scheduler run
  }

  public void setPivotSpeed(double speed){
    percentOutControlRequest.Output = speed;
    pivotMotor.setControl(percentOutControlRequest);
  }

  public void setPivotTarget(double targetAngle){
    targetAngle = Math.max(targetAngle, Constants.ShooterPivot.Limits.minPivotAngle);
    targetAngle = Math.min(targetAngle, Constants.ShooterPivot.Limits.minPivotAngle);

    pivotTarget = targetAngle;
  }

  public double getPivotTarget(){
    return pivotTarget;
  }

  public void setPivotToTarget(){
    double position = Math.max(pivotTarget, Constants.ShooterPivot.Limits.minPivotAngle);
    position = Math.min(position, Constants.ShooterPivot.Limits.minPivotAngle);

    if(Math.abs(getEncoderAngle() - position) > 15.0){
      pivotController.reset();
    }

    double power = pivotController.calculate(getEncoderAngle(), position) + Constants.ShooterPivot.PIDController.F;
    power = Math.min(power, .5);
    power = Math.max(power, -.5);

    percentOutControlRequest.Output = power;
    pivotMotor.setControl(percentOutControlRequest);
  }

  public double getEncoderAngle(){
    return pivotMedianFilter.calculate(lampreyEncoder.getAbsolutePosition());
  }

  public double getPivotAngle(){
    return getEncoderAngle();
  }

  public boolean atTarget(){
    return (Math.abs(getPivotAngle() - pivotTarget) < Constants.ShooterPivot.pivotTargetedThreshold);
  }
}
