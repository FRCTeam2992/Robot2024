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

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private TalonFX shooterMotor;

  private TalonFXConfiguration shooterMotorConfigs;

  private DutyCycleOut percentOutControlRequest;
  private VelocityDutyCycle velocityControlRequest;
  private MotionMagicVelocityDutyCycle motionMagicVelocityControlRequest;

  private double shooterTargetRPM = 200.0;

  private StatusSignal<Double> shooterVelocity;

  private MedianFilter medianFilter;

  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor = new TalonFX(Constants.Shooter.DeviceIDs.shooterMotorID);
    shooterMotorConfigs = new TalonFXConfiguration();

    shooterMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    shooterMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    if (!Constants.Shooter.ShooterPIDConstants.useCodePID) {
      // Make sure we copy old PID values in motor over so we don't lose them
      TalonFXConfiguration oldConfig = new TalonFXConfiguration();
      shooterMotor.getConfigurator().refresh(oldConfig); // Get the old config

      // Copy all slot 0 & MM paramters
      shooterMotorConfigs.Slot0.kA = oldConfig.Slot0.kA;
      shooterMotorConfigs.Slot0.kD = oldConfig.Slot0.kD;
      shooterMotorConfigs.Slot0.kG = oldConfig.Slot0.kG;
      shooterMotorConfigs.Slot0.kI = oldConfig.Slot0.kI;
      shooterMotorConfigs.Slot0.kP = oldConfig.Slot0.kP;
      shooterMotorConfigs.Slot0.kS = oldConfig.Slot0.kS;
      shooterMotorConfigs.Slot0.kV = oldConfig.Slot0.kV;
      shooterMotorConfigs.Slot0.StaticFeedforwardSign = oldConfig.Slot0.StaticFeedforwardSign;
      shooterMotorConfigs.Slot0.GravityType = oldConfig.Slot0.GravityType;
      shooterMotorConfigs.MotionMagic.MotionMagicAcceleration = oldConfig.MotionMagic.MotionMagicAcceleration;
      shooterMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = oldConfig.MotionMagic.MotionMagicCruiseVelocity;
      shooterMotorConfigs.MotorOutput.PeakForwardDutyCycle = oldConfig.MotorOutput.PeakForwardDutyCycle;
      shooterMotorConfigs.MotorOutput.PeakReverseDutyCycle = oldConfig.MotorOutput.PeakReverseDutyCycle;
    } else {
      shooterMotorConfigs.Slot0.kP = Constants.Shooter.ShooterPIDConstants.P;
      shooterMotorConfigs.Slot0.kI = Constants.Shooter.ShooterPIDConstants.I;
      shooterMotorConfigs.Slot0.kD = Constants.Shooter.ShooterPIDConstants.D;
      shooterMotorConfigs.Slot0.kV = Constants.Shooter.ShooterPIDConstants.V;
      shooterMotorConfigs.MotionMagic.MotionMagicAcceleration = Constants.Shooter.ShooterPIDConstants.acceleration;
      // shooterMotorConfigs.MotionMagic.MotionMagicCruiseVelocity =
      // Constants.Shooter.ShooterPIDConstants.cruiseVelocity;
      shooterMotorConfigs.MotionMagic.MotionMagicJerk = Constants.Shooter.ShooterPIDConstants.jerk;
    }

    shooterMotor.getConfigurator().apply(shooterMotorConfigs);

    percentOutControlRequest = new DutyCycleOut(0.0).withEnableFOC(true);
    velocityControlRequest = new VelocityDutyCycle(0.0).withEnableFOC(true).withSlot(0);
    motionMagicVelocityControlRequest = new MotionMagicVelocityDutyCycle(0.0).withEnableFOC(true).withSlot(0);

    shooterVelocity = shooterMotor.getVelocity();

    medianFilter = new MedianFilter(5);

  }

  @Override
  public void periodic() {
    SmartDashboard.putData(this);
    setShooterTargetRPM(SmartDashboard.getNumber("Set Shooter RPM", medianFilter.calculate(shooterTargetRPM)));
    SmartDashboard.putNumber("Shooter Target RPM", getShooterTargetRPM());
    SmartDashboard.putNumber("Shooter RPM", getShooterRPM());
    // This method will be called once per scheduler run
  }

  public void setShooterSpeed(double percentSpeed) {
    percentOutControlRequest.Output = percentSpeed;
    shooterMotor.setControl(percentOutControlRequest);
  }

  public void setShooterTargetRPM(double shooterSetRPM) {
    this.shooterTargetRPM = shooterSetRPM / 60.0;
  }

  public void setShooterToTargetRPM() {
    double speed = shooterTargetRPM;
    setShooterRawVelocity(speed);
  }

  public void setShooterRawVelocity(double velocity) {
    motionMagicVelocityControlRequest.Velocity = velocity;
    shooterMotor.setControl(motionMagicVelocityControlRequest);
  }

  public double getShooterRPM() {
    // return shooterVelocity.waitForUpdate(0.0).getValue() * 60.0;
    return shooterVelocity.refresh().getValue() * 60.0;
  }

  public double getShooterTargetRPM() {
    return shooterTargetRPM * 60;
  }

  public boolean atShooterRPM() {
    return (Math.abs(getShooterTargetRPM() - getShooterRPM()) < 200.0);
  }
}
