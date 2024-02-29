// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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

  private ProfiledPIDController profiledPivotController;
  private PIDController pivotController;
  
  private DutyCycleEncoder lampreyEncoder;

  private double pivotTarget;

  public MedianFilter pivotMedianFilter;

  private boolean holdPositionRecorded;
  private double holdPosition;

  private Constraints pidConstraints;

  /** Creates a new ShooterPivot. */
  public ShooterPivot() {

    pivotMotor = new TalonFX(Constants.ShooterPivot.pivotMotorID);
    pivotMotorConfigs = new TalonFXConfiguration();
    pivotMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pidConstraints = new TrapezoidProfile.Constraints(Constants.ShooterPivot.PIDController.maxVelocity, 
    Constants.ShooterPivot.PIDController.maxAcceleration);

    profiledPivotController = new ProfiledPIDController(Constants.ShooterPivot.PIDController.P, 
    Constants.ShooterPivot.PIDController.I, Constants.ShooterPivot.PIDController.D, pidConstraints);
    profiledPivotController.setTolerance(Constants.ShooterPivot.PIDController.pivotAngleTolerance);

    pivotController = new PIDController(Constants.ShooterPivot.PIDController.P, 
    Constants.ShooterPivot.PIDController.I, Constants.ShooterPivot.PIDController.D);
    profiledPivotController.setTolerance(Constants.ShooterPivot.PIDController.pivotAngleTolerance);

    lampreyEncoder = new DutyCycleEncoder(0);
    lampreyEncoder.setDistancePerRotation(1.0);

    percentOutControlRequest = new DutyCycleOut(0.0);

    pivotMedianFilter = new MedianFilter(3);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(this);
    SmartDashboard.putNumber("Pivot Angle (deg)", this.getEncoderAngle());
    // This method will be called once per scheduler run
  }

  public void setPivotSpeed(double speed){
    holdPositionRecorded = false;

    if (getEncoderAngle() < Constants.ShooterPivot.Limits.minPivotAngle 
    || getEncoderAngle() > Constants.ShooterPivot.Limits.maxPivotAngle) {
      speed = 0.0;
    }
    percentOutControlRequest.Output = speed;
    pivotMotor.setControl(percentOutControlRequest);
  }

  public void setPivotTarget(double targetAngle){
    targetAngle = Math.max(targetAngle, Constants.ShooterPivot.Limits.minPivotAngle);
    targetAngle = Math.min(targetAngle, Constants.ShooterPivot.Limits.maxPivotAngle);

    pivotTarget = targetAngle;
  }

  public double getPivotTarget(){
    return pivotTarget;
  }

  public void setPivotToTarget(){
    holdPositionRecorded = true;
    holdPosition = pivotTarget;

    double position = Math.max(pivotTarget, Constants.ShooterPivot.Limits.minPivotAngle);
    position = Math.min(position, Constants.ShooterPivot.Limits.maxPivotAngle);

    if(Math.abs(getEncoderAngle() - position) > 15.0){
      pivotController.reset();
    }

    double power = pivotController.calculate(getEncoderAngle(), position) + Constants.ShooterPivot.PIDController.F;
    power = Math.min(power, .5);
    power = Math.max(power, -.5);

    percentOutControlRequest.Output = power;
    pivotMotor.setControl(percentOutControlRequest);
  }

  public void holdPivot() {
    if (!holdPositionRecorded) {
      // We haven't recorded where we are yet, so get it
      holdPosition = getEncoderAngle();
      holdPositionRecorded = true;

      pivotMotor.set(0.0);
    } else {

    holdPosition = Math.max(pivotTarget, Constants.ShooterPivot.Limits.minPivotAngle);
    holdPosition = Math.min(holdPosition, Constants.ShooterPivot.Limits.maxPivotAngle);
    
    double power = pivotController.calculate(getEncoderAngle(), holdPosition) + Constants.ShooterPivot.PIDController.F;
    power = Math.min(power, .5);
    power = Math.max(power, -.5);

    percentOutControlRequest.Output = power;
    pivotMotor.setControl(percentOutControlRequest);
    }

  }

  public void setHoldPosition(double angle){
    holdPositionRecorded = true;

    holdPosition = angle;
  }

  public double getEncoderAngle(){
    return pivotMedianFilter.calculate(lampreyEncoder.getAbsolutePosition() * 360);
  }

  public double getPivotAngle(){
    return getEncoderAngle();
  }

  public boolean atTarget(){
    return (Math.abs(getPivotAngle() - pivotTarget) < Constants.ShooterPivot.pivotTargetedThreshold);
  }
}
