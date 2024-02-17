// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.StopElevator;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {
  /** Creates a new elevator. */
  private CANSparkMax leadMotor; //left
  private CANSparkMax followMotor1; //left
  private CANSparkMax followMotor2; //right
  private CANSparkMax followMotor3; //right

  private SparkPIDController PIDController;

  private boolean holdPositionRecorded;
  private double holdPosition;

  public Elevator() {
    leadMotor = new CANSparkMax(Constants.Elevator.leadMotorID, MotorType.kBrushless);
    followMotor1 = new CANSparkMax(Constants.Elevator.followMotor1ID, MotorType.kBrushless);
    followMotor2 = new CANSparkMax(Constants.Elevator.followMotor2ID, MotorType.kBrushless);
    followMotor3 = new CANSparkMax(Constants.Elevator.followMotor3ID, MotorType.kBrushless);

    leadMotor.setIdleMode(IdleMode.kBrake);
    followMotor1.setIdleMode(IdleMode.kBrake);
    followMotor2.setIdleMode(IdleMode.kBrake);
    followMotor3.setIdleMode(IdleMode.kBrake);

    leadMotor.setInverted(false);
    followMotor1.setInverted(false);
    followMotor2.setInverted(true);
    followMotor3.setInverted(true);

    followMotor1.follow(leadMotor, false);
    followMotor2.follow(leadMotor, true);
    followMotor3.follow(leadMotor, true);

    PIDController = leadMotor.getPIDController();
    PIDController.setP(Constants.Elevator.PIDConstants.kP);
    PIDController.setI(Constants.Elevator.PIDConstants.kI);
    PIDController.setD(Constants.Elevator.PIDConstants.kD);
    PIDController.setIZone(Constants.Elevator.PIDConstants.kIZone);
    PIDController.setOutputRange(Constants.Elevator.PIDConstants.kMinOutput, Constants.Elevator.PIDConstants.kMaxOutput);

    PIDController.setSmartMotionMaxVelocity(Constants.Elevator.PIDConstants.SmartMotionMaxVel, 0);
    PIDController.setSmartMotionMinOutputVelocity(Constants.Elevator.PIDConstants.SmartMotionMinVel, 0);
    PIDController.setSmartMotionMaxAccel(Constants.Elevator.PIDConstants.SmartMotionMaxAcc, 0);
    PIDController.setSmartMotionAllowedClosedLoopError(Constants.Elevator.PIDConstants.SmartMotionAllowedError, 0);



  }

  @Override
  public void periodic() {

    if (checkIfAtHardStop()){
      // zeroElevatorEncoders();
      // new StopElevator(this).schedule();
    }

    SmartDashboard.putNumber("Elevator Inches", getElevatorInches());
    SmartDashboard.putNumber("Elevator Motor Position", getElevatorPosition()[0]);
    SmartDashboard.putBoolean("At hard stop", checkIfAtHardStop());
    SmartDashboard.putData(this);

    SmartDashboard.putNumber("Ele Velocity", getElevatorVelocity());
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
    return rotations * Constants.Elevator.encoderToInches;
  }

  public double inchesToEncoderRotations(double inches) {
    return inches / Constants.Elevator.encoderToInches;
  }

  public void zeroElevatorEncoders() {
    leadMotor.getEncoder().setPosition(0.0);
    followMotor1.getEncoder().setPosition(0.0);
    followMotor2.getEncoder().setPosition(0.0);
    followMotor3.getEncoder().setPosition(0.0);
  }

  public void setElevatorSpeed(double speed) {
    holdPositionRecorded = false;

    // if (getElevatorInches() < Constants.Elevator.Limits.softStopBottom) {
    //   speed = Math.max(0.0, speed);
    // } else if (getElevatorInches() > Constants.Elevator.Limits.softStopTop) {
    //   speed = 0.0;
    // }

    leadMotor.set(speed);
  }

  public void setElevatorPosition(double position) {
    position = inchesToEncoderRotations(position);
    holdPositionRecorded = true;
    
    if (position < Constants.Elevator.Limits.softStopBottom) {
      position = Constants.Elevator.Limits.softStopBottom;
    } else if (position > Constants.Elevator.Limits.softStopTop) {
      position = Constants.Elevator.Limits.softStopTop;
    }

    holdPosition = position;

    PIDController.setReference(position, CANSparkMax.ControlType.kPosition, 0, Constants.Elevator.PIDConstants.kF);
    // leadMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
  }

  public boolean checkIfAtHardStop(){
    return false;
    // return (leadMotor.getOutputCurrent() > Constants.Elevator.Limits.hardStopCurrentLimit && leadMotor.getOutputCurrent() < 0.0);
  }

  public void holdElevator() {
    if (!holdPositionRecorded) {
      // We haven't recorded where we are yet, so get it
      holdPosition = getElevatorPosition()[0];
      holdPositionRecorded = true;

      leadMotor.set(0.0);
    } else {
    PIDController.setReference(holdPosition, CANSparkMax.ControlType.kSmartMotion);
    }

  }

  public boolean atPosition(){
    return (encoderRotationsToInches(Math.abs(holdPosition - getElevatorPosition()[0])) < Constants.Elevator.elevatorHeightToleranceInch);
  }

  public double getElevatorVelocity(){
    return leadMotor.getEncoder().getVelocity();
  }

}
