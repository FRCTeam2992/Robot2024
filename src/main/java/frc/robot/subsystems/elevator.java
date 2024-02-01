// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class elevator extends SubsystemBase {
  /** Creates a new elevator. */
  private CANSparkMax leadMotor; //left
  private CANSparkMax followMotor1; //left
  private CANSparkMax followMotor2; //right
  private CANSparkMax followMotor3; //right

  private PIDController PIDController;

  public elevator() {
    leadMotor = new CANSparkMax(11, MotorType.kBrushless);
    followMotor1 = new CANSparkMax(10, MotorType.kBrushless);
    followMotor2 = new CANSparkMax(13, MotorType.kBrushless);
    followMotor3 = new CANSparkMax(12, MotorType.kBrushless);

    leadMotor.setIdleMode(IdleMode.kBrake);
    followMotor1.setIdleMode(IdleMode.kBrake);
    followMotor2.setIdleMode(IdleMode.kBrake);
    followMotor3.setIdleMode(IdleMode.kBrake);

    leadMotor.setInverted(true);
    followMotor1.setInverted(true);
    followMotor2.setInverted(true);
    followMotor3.setInverted(true);

    followMotor1.follow(leadMotor);
    followMotor2.follow(leadMotor);
    followMotor3.follow(leadMotor);

    PIDController = new PIDController(0.0, 0.0, 0.0); //0.0s are placeholder values
    PIDController.setTolerance(0.0); //placeholder
    PIDController.disableContinuousInput();
    PIDController.setIntegratorRange(-0.2, 0.2); //placeholder (copied from Rodrigue's code)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double[] getElevatorPosition() {
      double[] positions = {0.0, 0.0, 0.0, 0.0}; //{0: lead, 1: follow1, 2: follow2, 3: follow3}
      positions[0] = leadMotor.getEncoder().getPosition();
      positions[1] = followMotor1.getEncoder().getPosition();
      positions[2] = followMotor2.getEncoder().getPosition();
      positions[3] = followMotor3.getEncoder().getPosition();
      return positions;
  }

  public double getElevatorInches() {
    return encoderRotationsToInches(getElevatorPosition()[0]);
  }

  public double encoderRotationsToInches(double rotations) {
    return rotations * 0.0; //Constants.Elevator.encoderElevatorToInches;
  }

  public void zeroElevatorEncoders() {
    leadMotor.getEncoder().setPosition(0.0);
    followMotor1.getEncoder().setPosition(0.0);
    followMotor2.getEncoder().setPosition(0.0);
    followMotor3.getEncoder().setPosition(0.0);
  }

  public void setElevatorSpeed(double speed) {
    if (getElevatorInches() < 4.0 && speed < 0.0) { //4 inch leeway is temp
      speed = Math.max(speed, -0.5); //limits to a slow speed of .5 or slower
    }
    leadMotor.set(speed);
    followMotor1.set(speed);
    followMotor2.set(speed);
    followMotor3.set(speed);
  }

  public void setElevatorPosition(double position) {
    if (position < 0.0) {
      position = 0.0;
    }

    leadMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
    followMotor1.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
    followMotor2.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
    followMotor3.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
  }
}
