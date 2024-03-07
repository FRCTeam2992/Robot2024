// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SetElevatorTargetPosition;
import frc.robot.commands.StopElevator;
import frc.robot.Constants;
import frc.robot.MyRobotState;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {

  private MyRobotState mRobotState;
  /** Creates a new elevator. */
  private CANSparkMax leadMotor; //left
  private CANSparkMax followMotor1; //left
  private CANSparkMax followMotor2; //right
  private CANSparkMax followMotor3; //right

  private SparkPIDController PIDController;

  private double targetPosition;
  private boolean holdPositionRecorded;
  private double holdPosition;
  
  private enum ElevatorModeState {
    Stopped,
    Hold,
    PIDPosition,
    ManualMovement
  }

  private ElevatorModeState elevatorMode = ElevatorModeState.Stopped;

  public Elevator(MyRobotState robotState) {

    mRobotState = robotState;

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

    leadMotor.setClosedLoopRampRate(0.3);
    followMotor1.setClosedLoopRampRate(0.3);
    followMotor2.setClosedLoopRampRate(0.3);
    followMotor3.setClosedLoopRampRate(0.3);

    followMotor1.follow(leadMotor, false);
    followMotor2.follow(leadMotor, true);
    followMotor3.follow(leadMotor, true);

    PIDController = leadMotor.getPIDController();
    PIDController.setP(Constants.Elevator.PIDConstants.kP0, 0);
    PIDController.setI(Constants.Elevator.PIDConstants.kI0, 0);
    PIDController.setD(Constants.Elevator.PIDConstants.kD0, 0);
    PIDController.setIZone(Constants.Elevator.PIDConstants.kIZone0, 0);
    PIDController.setOutputRange(Constants.Elevator.PIDConstants.kMinOutput, Constants.Elevator.PIDConstants.kMaxOutput, 0);
    
    PIDController.setP(Constants.Elevator.PIDConstants.kP1, 1);
    PIDController.setI(Constants.Elevator.PIDConstants.kI1, 1);
    PIDController.setD(Constants.Elevator.PIDConstants.kD1, 1);
    PIDController.setIZone(Constants.Elevator.PIDConstants.kIZone1, 1);
    PIDController.setOutputRange(Constants.Elevator.PIDConstants.kMinOutput, Constants.Elevator.PIDConstants.kMaxOutput, 0);


  }

  @Override
  public void periodic() {


    if (checkIfAtHardStop()){
      // zeroElevatorEncoders();
      // new HoldElevator(this).schedule();
    }
    // setElevatorTargetPosition(SmartDashboard.getNumber("Set Elevator Position",
    // 0.0));

    SmartDashboard.putNumber("Elevator Inches", getElevatorInches());
    SmartDashboard.putNumber("Elevator Motor Position", getElevatorPosition()[0]);
    SmartDashboard.putBoolean("At hard stop", checkIfAtHardStop());
    SmartDashboard.putData(this);

    SmartDashboard.putNumber("Ele Velocity", getElevatorVelocity());

    SmartDashboard.putNumber("Elevator target", getTargetPosition());
    // This method will be called once per scheduler run
    if (DriverStation.isDisabled()) {
      setElevatorSpeed(0.0);
    }

    // Here we do the work for all modes except manual control
    switch (elevatorMode) {
      case Stopped: {
        // Stop the motor!
        setElevatorSpeed(0.0);
        break;
      }
      case Hold: {
        // Hold current position using PID

        // And actually make the motor hold
        holdElevator();
        break;
      }

      case PIDPosition: {
        // Doing PID control so move it
        moveElevatorToTargetPosition();
        break;
      }

      case ManualMovement: {
        // We are in manual override mode. Periodic will do nothing. Let manual command
        // drive
        break;
      }
    }
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
    elevatorMode = ElevatorModeState.ManualMovement;
    holdPositionRecorded = false;

    switch (mRobotState.getRobotMode()) {

      case Auto: {
        // Do nothing -- shouldn't ever be calling for manual moves in Auto mode
        elevatorMode = ElevatorModeState.Stopped;
        speed = 0.0;
        break;
      }

      case DefaultSpeaker:
      case Speaker: {
        // Enforce soft top and bottom limits
        if (getElevatorInches() > Constants.Elevator.Limits.maxElevatorSpeaker && speed > 0.0) {
          speed = 0.0; // Set to minimum down speed since encoder says too low
        } 
        break;
      }

      case Amp: {
        break;
      }

      case Endgame: {
        break;
      }

      case Override: {
        leadMotor.set(speed);
        return;
      }
    }

    if (getElevatorInches() < Constants.Elevator.Limits.softStopBottom) {
      speed = Math.max(-0.1, speed);
    } else if (getElevatorInches() > Constants.Elevator.Limits.softStopTop) {
      speed = Math.min(0.0, speed);
    }

    leadMotor.set(speed);
  }

  public void setElevatorTargetPosition(double position) {

    switch (mRobotState.getRobotMode()) {
      case Override:
      case DefaultSpeaker:
      case Speaker:
      case Auto: {
        // In these cases use normal top limit
        position = Math.min(position, Constants.Elevator.Limits.maxElevatorSpeaker);
        break;
      }
      case Amp: {
        position = Math.min(position, Constants.Elevator.Limits.maxElevatorAmp);
        break;
      }
      case Endgame: {
        position = Math.min(position, Constants.Elevator.Limits.hardStopTop);
        break;
      }

    }
    
    double mPosition = Math.max(position, Constants.Elevator.Limits.softStopBottom);
    mPosition = Math.min(mPosition, Constants.Elevator.Limits.softStopTop);

    targetPosition = inchesToEncoderRotations(mPosition);
  }

  public void moveElevatorToTargetPosition() {
    if (mRobotState.IsOverrideMode()) {
      // Should never call this but just in case
      elevatorMode = ElevatorModeState.Stopped;
      setElevatorSpeed(0.0);
      return;
    }

    double position = targetPosition;

    switch (mRobotState.getRobotMode()) {
      case Override:
      case DefaultSpeaker:
      case Speaker:
      case Auto: {
        // In these cases use normal top limit
        position = Math.min(position, Constants.Elevator.Limits.maxElevatorSpeaker);
        break;
      }
      case Amp: {
        position = Math.min(position, Constants.Elevator.Limits.maxElevatorAmp);
        break;
      }
      case Endgame: {
        position = Math.min(position, Constants.Elevator.Limits.hardStopTop);
        break;
      }

    }

    holdPositionRecorded = true;

    holdPosition = position;

    PIDController.setReference(position, CANSparkMax.ControlType.kPosition, 0, Constants.Elevator.PIDConstants.kF0);
    // PIDController.setReference(position, ControlType.kSmartMotion);

    // leadMotor.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
  }

  public boolean checkIfAtHardStop(){
    // return false;
     return (Robot.mRobotContainer.mPDH.getCurrent(Constants.Elevator.leadMotorPDHPort) > Constants.Elevator.Limits.hardStopCurrentLimit && leadMotor.getOutputCurrent() < 0.0);
  }

  public void holdElevator() {
    if (mRobotState.IsOverrideMode()) {
      // Should never call this but just in case
      elevatorMode = ElevatorModeState.Stopped;
      setElevatorSpeed(0.0);
      return;
    }

    if (!holdPositionRecorded) {
      // We haven't recorded where we are yet, so get it
      holdPosition = getElevatorPosition()[0];
      holdPositionRecorded = true;

      leadMotor.set(0.0);
    } else {
      holdPosition = Math.max(holdPosition, 0.0);
      holdPosition = Math.min(holdPosition, Constants.Elevator.Limits.hardStopTop);

    PIDController.setReference(holdPosition, CANSparkMax.ControlType.kPosition, 0, Constants.Elevator.PIDConstants.kF0);
    }
 
  }

  public boolean atPosition(){
    return (encoderRotationsToInches(Math.abs(targetPosition - getElevatorPosition()[0])) < Constants.Elevator.elevatorHeightToleranceInch);
  }

  public double getElevatorVelocity(){
    return leadMotor.getEncoder().getVelocity();
  }

  public double getTargetPosition(){
    return encoderRotationsToInches(targetPosition) ;
  }

  public void setElevatorVelocity( double velocity){
    holdPositionRecorded = false;

    switch (mRobotState.getRobotMode()) {
      
      case Override:
      case Auto: {
        // Do nothing -- shouldn't ever be calling for manual moves in Auto mode
        elevatorMode = ElevatorModeState.Stopped;
        velocity = 0.0;
        break;
      }

      case DefaultSpeaker:
      case Speaker: {
        // Enforce soft top and bottom limits
        if (getElevatorInches() > Constants.Elevator.Limits.maxElevatorSpeaker && velocity > 0.0) {
          velocity = 0.0; // Set to minimum down speed since encoder says too low
        } 
        break;
      }

      case Amp: {
        break;
      }

      case Endgame: {
        break;
      }
    }

    if (getElevatorInches() <= 0.15) {
      velocity = Math.max(0.0, velocity);
    } else if (getElevatorInches() > Constants.Elevator.Limits.softStopTop) {
      velocity = 0.0;
    }
    if (getElevatorInches() < Constants.Elevator.Limits.softStopBottom){
      velocity = 0.5;
    }

    velocity = inchesToEncoderRotations(velocity);

    PIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity, 1);

  }

  public double getRobotHeight(double pivotAngle){
    double height = Constants.ShooterPivot.pivotHeight + getElevatorInches() 
    + (Constants.ShooterPivot.shooterLength * Math.sin(pivotAngle + 19.537819)) + Constants.ShooterPivot.flyWheelRadius;
    return (height);
  }

  public boolean isAboveHeightLimit(double pivotAngle){
    return (getRobotHeight(pivotAngle) > Constants.ShooterPivot.robotMaxHeight);
  }

  public void setHoldPositionRecorded(boolean isRecorded){
    holdPositionRecorded = isRecorded;
  }

  private void setHoldPosition(double position) {
    // Doesn't do any bounds checks on limits! Use cautiously
    holdPositionRecorded = true;
    holdPosition = position;
  }

  public void setElevatorHoldMode() {
    elevatorMode = ElevatorModeState.Hold;
  }

  public void setElevatorPIDMode(){
    elevatorMode = ElevatorModeState.PIDPosition;
  }

  public void setElevatorManualMode(){
    elevatorMode = ElevatorModeState.ManualMovement;
  }

  


}
