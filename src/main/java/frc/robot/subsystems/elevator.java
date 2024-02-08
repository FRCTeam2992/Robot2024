// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants;
import frc.robot.commands.StopElevator;
import frc.robot.commands.moveElevator;

public class Elevator extends SubsystemBase {
  /** Creates a new elevator. */
  private CANSparkMax leadMotor; //left
  private CANSparkMax followMotor1; //left
  private CANSparkMax followMotor2; //right
  private CANSparkMax followMotor3; //right

  private PIDController PIDController;

  private boolean holdPositionRecorded;
  private double holdPosition;

  public Elevator() {
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

    PIDController = new PIDController(Constants.Elevator.PIDControllerP, Constants.Elevator.PIDControllerI, Constants.Elevator.PIDControllerD);
    PIDController.setTolerance(Constants.Elevator.positionTolerance);
    PIDController.disableContinuousInput();
    PIDController.setIntegratorRange(Constants.Elevator.integratorRangeMin, Constants.Elevator.integratorRangeMax);
  }

  @Override
  public void periodic() {

    if (checkIfAtHardStop()){
      zeroElevatorEncoders();
      new StopElevator(this).schedule();
    }

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
    return rotations * Constants.Elevator.encoderElevatorToInches;
  }

  public void zeroElevatorEncoders() {
    leadMotor.getEncoder().setPosition(0.0);
    followMotor1.getEncoder().setPosition(0.0);
    followMotor2.getEncoder().setPosition(0.0);
    followMotor3.getEncoder().setPosition(0.0);
  }

  public void setElevatorSpeed(double speed) {
    holdPositionRecorded = false;

    if (getElevatorInches() < Constants.Elevator.Limits.softStopBottom) {
      speed = Math.max(-0.1, speed);
    } else if (getElevatorInches() > Constants.Elevator.Limits.softStopTop) {
      speed = 0.0;
    }

    leadMotor.set(speed);
  }

  public void setElevatorPosition(double position) {
    holdPositionRecorded = true;
    
    if (position < Constants.Elevator.Limits.softStopBottom) {
      position = Constants.Elevator.Limits.softStopBottom;
    } else if (position > Constants.Elevator.Limits.softStopTop) {
      position = Constants.Elevator.Limits.softStopTop;
    }

    holdPosition = position;

    leadMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
  }

  public boolean checkIfAtHardStop(){
    return (leadMotor.getOutputCurrent() > Constants.Elevator.Limits.hardStopCurrentLimit && leadMotor.getOutputCurrent() < 0.0);
  }

  public void holdElevator() {
    if (!holdPositionRecorded) {
      // We haven't recorded where we are yet, so get it
      holdPosition = getElevatorInches();
      holdPositionRecorded = true;

      leadMotor.set(0.0);
    } else {
    leadMotor.getPIDController().setReference(holdPosition, CANSparkMax.ControlType.kPosition);
    }

  }

  public boolean atPosition(){
    return (Math.abs(holdPosition - getElevatorInches()) < Constants.Elevator.elevatorHeightToleranceInch);
  }
}
