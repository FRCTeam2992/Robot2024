// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MyRobotState;

public class ShooterPivot extends SubsystemBase {
  
  private MyRobotState mState;
  
  private TalonFX pivotMotor;
  private TalonFXConfiguration pivotMotorConfigs;
  
  private DutyCycleOut percentOutControlRequest;
  private MotionMagicDutyCycle motionMagicControlRequest;
  
  private ProfiledPIDController profiledPivotController;
  private PIDController pivotController;
  
  // private DutyCycleEncoder lampreyEncoder;
  private double currentPivotAngle;  // Update from encoder via filter once per cycle
  
  private double pivotTarget;
  
  private MedianFilter pivotMedianFilter;
  
  private boolean holdPositionRecorded;
  private double holdPosition;
  
  private Constraints pidConstraints;
  
  private enum PivotModeState {
    Stopped,
    Hold,
    PIDMovement,
    ManualMovement
  }
  
  private PivotModeState pivotMode = PivotModeState.Stopped;
  
  /** Creates a new ShooterPivot. */
  public ShooterPivot(MyRobotState state) {
    mState = state;
    
    pivotMotor = new TalonFX(Constants.ShooterPivot.pivotMotorID);
    pivotMotorConfigs = new TalonFXConfiguration();
    pivotMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    if (!Constants.ShooterPivot.PIDController.useCodePID) {
      // Make sure we copy old PID values in motor over so we don't lose them
      TalonFXConfiguration oldConfig = new TalonFXConfiguration();
      pivotMotor.getConfigurator().refresh(oldConfig); // Get the old config

      // Copy all slot 0 & MM paramters
      pivotMotorConfigs.Slot0.kA = oldConfig.Slot0.kA;
      pivotMotorConfigs.Slot0.kD = oldConfig.Slot0.kD;
      pivotMotorConfigs.Slot0.kG = oldConfig.Slot0.kG;
      pivotMotorConfigs.Slot0.kI = oldConfig.Slot0.kI;
      pivotMotorConfigs.Slot0.kP = oldConfig.Slot0.kP;
      pivotMotorConfigs.Slot0.kS = oldConfig.Slot0.kS;
      pivotMotorConfigs.Slot0.kV = oldConfig.Slot0.kV;
      pivotMotorConfigs.Slot0.StaticFeedforwardSign = oldConfig.Slot0.StaticFeedforwardSign;
      pivotMotorConfigs.Slot0.GravityType = oldConfig.Slot0.GravityType;
      pivotMotorConfigs.MotionMagic.MotionMagicAcceleration = oldConfig.MotionMagic.MotionMagicAcceleration;
      pivotMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = oldConfig.MotionMagic.MotionMagicCruiseVelocity;
      pivotMotorConfigs.MotorOutput.PeakForwardDutyCycle = oldConfig.MotorOutput.PeakForwardDutyCycle;
      pivotMotorConfigs.MotorOutput.PeakReverseDutyCycle = oldConfig.MotorOutput.PeakReverseDutyCycle;
    } else {
      pivotMotorConfigs.Slot0.kP = Constants.ShooterPivot.PIDController.P;
      pivotMotorConfigs.Slot0.kI = Constants.ShooterPivot.PIDController.I;
      pivotMotorConfigs.Slot0.kD = Constants.ShooterPivot.PIDController.D;
      pivotMotorConfigs.Slot0.kS = Constants.ShooterPivot.PIDController.F;
      pivotMotorConfigs.MotionMagic.MotionMagicAcceleration = Constants.ShooterPivot.PIDController.maxAcceleration;
      pivotMotorConfigs.MotionMagic.MotionMagicCruiseVelocity = Constants.ShooterPivot.PIDController.maxVelocity;
      pivotMotorConfigs.MotorOutput.PeakForwardDutyCycle = Constants.ShooterPivot.PIDController.peakForwardDutyCycle;
      pivotMotorConfigs.MotorOutput.PeakReverseDutyCycle = Constants.ShooterPivot.PIDController.peakReverseDutyCycle;
    }

    pivotMotor.getConfigurator().apply(pivotMotorConfigs);
    
    pidConstraints = new TrapezoidProfile.Constraints(Constants.ShooterPivot.PIDController.maxVelocity, 
    Constants.ShooterPivot.PIDController.maxAcceleration);
    
    profiledPivotController = new ProfiledPIDController(Constants.ShooterPivot.PIDController.P, 
    Constants.ShooterPivot.PIDController.I, Constants.ShooterPivot.PIDController.D, pidConstraints);
    profiledPivotController.setTolerance(Constants.ShooterPivot.PIDController.pivotAngleTolerance);
    
    pivotController = new PIDController(Constants.ShooterPivot.PIDController.P, 
    Constants.ShooterPivot.PIDController.I, Constants.ShooterPivot.PIDController.D);
    profiledPivotController.setTolerance(Constants.ShooterPivot.PIDController.pivotAngleTolerance);
    
    // lampreyEncoder = new DutyCycleEncoder(0);
    // lampreyEncoder.setDistancePerRotation(1.0);
    
    percentOutControlRequest = new DutyCycleOut(0.0).withEnableFOC(true);
    motionMagicControlRequest = new MotionMagicDutyCycle(0.0).withEnableFOC(true).withSlot(0);
    
    pivotMedianFilter = new MedianFilter(3);
    
    pivotMode = PivotModeState.Stopped;
  }
  
  /*
  * Periodic will do most of the work, obeying the subystem Mode to decide what to do.  The only alternative is if we go into
  * manual control mode.
  * 
  */
  @Override
  public void periodic() {
    // Update the current encoder reading only once per cycle here in Periodic for moving average
    currentPivotAngle = getPivotAngle();
    setPivotTarget(SmartDashboard.getNumber("Set Pivot angle", 0.0));
    SmartDashboard.putNumber("Pivot target angle", getPivotTarget());
    
    SmartDashboard.putData(this);
    SmartDashboard.putNumber("Shooter Pivot Angle (deg)", currentPivotAngle);
    // This method will be called once per scheduler run
    
    
    // Here we do the work for all modes except manual control
    switch (pivotMode) {
      case Stopped: {
        // Stop the motor!
        stopPivotMotor();
        break;
      }
      case Hold: {
        // Hold current position using PID
        
        // And actually make the motor hold
        holdPivotMotor();
        break;
      }
      
      case PIDMovement: {
        // Doing PID control so move it
        setPivotToTarget();
        break;
      }
      case ManualMovement: {
        // We are in manual override mode.  Periodic will do nothing.  Let manual command drive
        break;
      }
    }
    
  }
  
  private void stopPivotMotor() {
    // Called only fro periodic
    holdPositionRecorded = false;
    percentOutControlRequest.Output = 0.0;
    pivotMotor.setControl(percentOutControlRequest);
  }
  
  
  private void setPivotToTarget(){
    // Called only from periodic
    if (mState.IsOverrideMode()) {
      // Should never call this but just in case
      pivotMode = PivotModeState.Stopped;
      stopPivotMotor();
    }
    else {
      // Validate the target position against current robot state
      double position = Math.max(pivotTarget, Constants.ShooterPivot.Limits.minPivotAngle);
      switch (mState.getRobotMode()) {
        case Override:
        case DefaultSpeaker:
        case Speaker:
        case Auto: {
          // In these cases use normal top limit
          position = Math.min(position, Constants.ShooterPivot.Limits.maxPivotAngle);
          break;
        }
        case Amp: {
          position = Math.min(position, Constants.ShooterPivot.Limits.maxPivotAmp);
          break;
        }
        case Endgame: {
          position = Math.min(position, Constants.ShooterPivot.Limits.maxPivotEndgame);
          break;
        }
      }
      
      // Reset the I term of pivot if too large an error
      if(Math.abs(currentPivotAngle - position) > 5.0){
        pivotController.reset();
      }
      
      // double power = pivotController.calculate(currentPivotAngle, position) + Constants.ShooterPivot.PIDController.F;
      // power = Math.min(power, .40);
      // power = Math.max(power, -.25);

      
      holdPositionRecorded = true;
      holdPosition = position;
      
      // Reset the pivot target after constraint checks
      pivotTarget = position;
      
      // percentOutControlRequest.Output = power;
      // pivotMotor.setControl(percentOutControlRequest);

      motionMagicControlRequest.Position = (position / 360) * Constants.ShooterPivot.pivotGearRatio;
      pivotMotor.setControl(motionMagicControlRequest);
    }
  }
  
  
  private void holdPivotMotor() {
    // Called only from periodic
    updateHoldPosition(); // Will check for limits if mode changed!
    
    // double power = pivotController.calculate(currentPivotAngle, holdPosition) + Constants.ShooterPivot.PIDController.F;
    // power = Math.min(power, .15);
    // power = Math.max(power, -.06);
    
    // percentOutControlRequest.Output = power;
    // pivotMotor.setControl(percentOutControlRequest);
    motionMagicControlRequest.Position = (holdPosition / 360) * Constants.ShooterPivot.pivotGearRatio;
    pivotMotor.setControl(motionMagicControlRequest);
  }
  
  /*
  * And here are the public methods commands should use to control us.
  * 
  */
  public void setPivotSpeed(double speed){
    holdPositionRecorded = false;
    
    switch (mState.getRobotMode()) {
      case Auto: {
        // Do nothing -- shouldn't ever be calling for manual moves in Auto mode
        pivotMode = PivotModeState.Stopped;
        speed = 0.0;
        break;
      }
      
      case Override: {
        // Protections are off -- maybe a bad encoder.  Let speed go through unchecked
        pivotMode = PivotModeState.ManualMovement;
        break;
      }
      
      case DefaultSpeaker:      
      case Speaker: {
        // Enforce soft top and bottom limits
        if (currentPivotAngle < Constants.ShooterPivot.Limits.minPivotAngle && speed < 0.0) {
          speed = -0.02;  // Set to minimum down speed since encoder says too low
        }
        else if (currentPivotAngle > Constants.ShooterPivot.Limits.maxPivotAngle && speed > 0.0) {
          speed = 0.0; // Set to miniumum up speed since encoder says too high
        }
        break;
      }
      
      case Amp: {
        if (currentPivotAngle < Constants.ShooterPivot.Limits.minPivotAngle && speed < 0.0) {
          speed = -0.01;  // Set to minimum down speed since encoder says too low
        }
        else if (currentPivotAngle > Constants.ShooterPivot.Limits.maxPivotAmp && speed > 0.0) {
          speed = 0.0; // Can't go up anymore since in Amp mode and too high
        }
        break;
      }
      
      case Endgame: {
        if (currentPivotAngle < Constants.ShooterPivot.Limits.minPivotAngle && speed < 0.0) {
          speed = -0.01;  // Set to minimum down speed since encoder says too low
        }
        else if (currentPivotAngle > Constants.ShooterPivot.Limits.maxPivotEndgame && speed > 0.0) {
          speed = 0.0; // Can't go up anymore since in Endgame Mode and too high
        }
        break;
      }
    }
    
    percentOutControlRequest.Output = speed;
    pivotMotor.setControl(percentOutControlRequest);
  }
  
  // Set the PID target but don't change modes
  public void setPivotTarget(double targetAngle){
    // make sure we pick a good target value
    targetAngle = Math.max(targetAngle, Constants.ShooterPivot.Limits.minPivotAngle);
    switch (mState.getRobotMode()) {
      case Override:
      case DefaultSpeaker:      
      case Auto:
      case Speaker: {
        // Just do the "normal" checks in these modes  
        targetAngle = Math.min(targetAngle, Constants.ShooterPivot.Limits.maxPivotAngle);
        break;
      }
      case Amp: {
        // Apply the lower Amp mode limit on target angle
        targetAngle = Math.min(targetAngle, Constants.ShooterPivot.Limits.maxPivotAmp);
        break;
      }
      case Endgame: {
        targetAngle = Math.min(targetAngle, Constants.ShooterPivot.Limits.maxPivotEndgame);
        break;
      }
    }
    
    // And set the pivot target (but don't change modes)
    pivotTarget = targetAngle;
  }
  
  
  public double getPivotTarget(){
    return pivotTarget;
  }
  
  // Start the PID
  public void setPivotToPID() {
    pivotMode = PivotModeState.PIDMovement;
    holdPosition = pivotTarget;
    holdPositionRecorded = true;
  }
  
  public void updateHoldPosition() {
    if (!holdPositionRecorded) {
      // We don't already have a position so grab current position as target hold position
      setHoldPosition(currentPivotAngle);
    }
    
    // Now check if the hold position is valid for current mode
    double angle = Math.max(holdPosition, Constants.ShooterPivot.Limits.minPivotAngle);
    switch (mState.getRobotMode()) {
      case Auto:
      case DefaultSpeaker:      
      case Speaker:
      case Override: {
        // Just use the normal hold to current position -- don't need to do anything
        angle = Math.min(angle, Constants.ShooterPivot.Limits.maxPivotAngle);
        break;
      }
      case Amp: {
        angle = Math.min(angle, Constants.ShooterPivot.Limits.maxPivotAmp);
        break;
      }
      case Endgame: {
        angle = Math.min(angle, Constants.ShooterPivot.Limits.maxPivotEndgame);
      }
    }
    
    this.setHoldPosition(angle);
    
  }
  
  private void setHoldPosition(double angle){
    // Doesn't do any bounds checks on limits!  Use cautiously
    holdPositionRecorded = true;
    holdPosition = angle;
  }

  public void setPivotHoldMode() {
    updateHoldPosition();
    pivotMode = PivotModeState.Hold;
  }
  
  public void stopPivot() {
    pivotMode = PivotModeState.Stopped;
  }
  
  
  public double getPivotAngle(){
    // return pivotMedianFilter.calculate(lampreyEncoder.getAbsolutePosition() * 360);

    return pivotMotor.getPosition().getValueAsDouble() / Constants.ShooterPivot.pivotGearRatio * 360;
  }
  
  public boolean atTarget(){
    return (Math.abs(currentPivotAngle - pivotTarget) < Constants.ShooterPivot.pivotTargetedThreshold);
  }

  public void zeroPivotEncoder(){
    pivotMotor.setPosition(0.0);
  }
  
}