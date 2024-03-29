// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MyRobotState;
import frc.robot.MyRobotState.LEDModeState;
import frc.robot.MyRobotState.RobotModeState;
import frc.robot.MyRobotState.RobotNoteLocation;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;

  public RobotNoteLocation noteLocation = RobotNoteLocation.noNote;
  public boolean isOnTarget = false;

  public LEDModeState mLEDMode = LEDModeState.idle;
  public RobotModeState mRobotMode = RobotModeState.Speaker;

  private MyRobotState mRobotState;

  public LEDs(MyRobotState robotState) {
    m_led = new AddressableLED(0); //~
    m_ledBuffer = new AddressableLEDBuffer(17); //~ Copied from Stingray's code
    m_led.setLength(m_ledBuffer.getLength()); //~

    m_led.setData(m_ledBuffer);
    m_led.start();

    mRobotState = robotState;
  }

  @Override
  public void periodic() {
    if (mLEDMode == LEDModeState.intaking && noteLocation != RobotNoteLocation.noNote ) {
      mRobotState.setLEDMode(LEDModeState.idle);
    }

    if (mLEDMode != mRobotState.getLEDMode() || noteLocation != mRobotState.getNoteLocation()
    ||  isOnTarget != mRobotState.isOnTarget()){
      
      noteLocation = mRobotState.getNoteLocation();
      isOnTarget = mRobotState.isOnTarget();
      mLEDMode = mRobotState.getLEDMode();
      mRobotMode = mRobotState.getRobotMode();
      // hasNoteInIntake
      
      switch (mRobotState.getLEDMode()) {
        case idle: {
          break;
        }
        case intaking: {
          break;
        }
        case aiming: {
          break;
        }
        case shooting: {
          break;
        }    
      }
    }
    
    // This method will be called once per scheduler run
  }

  public void setSingleLEDColor(int pixel, Color color) {
      m_ledBuffer.setLED(pixel, color);
  }

  public void setLEDIntakingColorChase(Color color){

  }

  public void setLEDStripColor(Color color) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        setSingleLEDColor(i, color);
    }

    m_led.setData(m_ledBuffer);
  }

  public void setLEDNoNote(Color color) {

  }

  public void setLEDIdleMode(){
    switch (mRobotMode){
      case Override:{
        setLEDStripColor(Constants.LEDs.Colors.override);
        break;
      }

      case Speaker:{
        switch (noteLocation){
          case intake:
          case shooter: {
            setLEDStripColor(Constants.LEDs.Colors.speaker);
            break;
          }
          case noNote:{
            setLEDNoNote(Constants.LEDs.Colors.speaker);
            break;
          }
        }
        
        break;
      }

      case Amp:{
        switch (noteLocation){
          case intake:
          case shooter: {
            setLEDStripColor(Constants.LEDs.Colors.amp);
            break;
          }
          case noNote:{
            setLEDNoNote(Constants.LEDs.Colors.amp);
            break;
          }
        }
        break;
      }

      case Endgame:{
        switch (noteLocation){
          case intake:
          case shooter: {
            setLEDStripColor(Constants.LEDs.Colors.endgame);
            break;
          }
          case noNote:{
            setLEDNoNote(Constants.LEDs.Colors.endgame);
            break;
          }
        }
        break;
      }

      case DefaultSpeaker:{
        switch (noteLocation){
          case intake:
          case shooter: {
            setLEDStripColor(Constants.LEDs.Colors.defaultSpeaker);
            break;
          }
          case noNote:{
            setLEDNoNote(Constants.LEDs.Colors.defaultSpeaker);
            break;
          }
        }
        break;
      }

      case Passing: {
        switch (noteLocation){
          case intake:
          case shooter: {
            setLEDStripColor(Constants.LEDs.Colors.passing);
            break;
          }
          case noNote:{
            setLEDNoNote(Constants.LEDs.Colors.passing);
            break;
          }
        }
        break;
      }

      case Auto:
      break;
    }
  }

  public void setLEDIntakingMode(){
    switch (mRobotMode){
      case Override:{
        setLEDStripColor(Constants.LEDs.Colors.override);
        break;
      }

      case Speaker:{
        switch (noteLocation){
          case intake:
          case shooter: {
            setLEDStripColor(Constants.LEDs.Colors.speaker);
            break;
          }
          case noNote:{
            setLEDIntakingColorChase(Constants.LEDs.Colors.speaker);
            break;
          }
        }
        
        break;
      }

      case Amp:{
        switch (noteLocation){
          case intake:
          case shooter: {
            setLEDStripColor(Constants.LEDs.Colors.amp);
            break;
          }
          case noNote:{
            setLEDIntakingColorChase(Constants.LEDs.Colors.amp);
            break;
          }
        }
        break;
      }

      case Endgame:{
        switch (noteLocation){
          case intake:
          case shooter: {
            setLEDStripColor(Constants.LEDs.Colors.endgame);
            break;
          }
          case noNote:{
            setLEDIntakingColorChase(Constants.LEDs.Colors.endgame);
            break;
          }
        }
        break;
      }

      case DefaultSpeaker:{
        switch (noteLocation){
          case intake:
          case shooter: {
            setLEDStripColor(Constants.LEDs.Colors.defaultSpeaker);
            break;
          }
          case noNote:{
            setLEDIntakingColorChase(Constants.LEDs.Colors.defaultSpeaker);
            break;
          }
        }
        break;
      }

      case Passing: {
        switch (noteLocation){
          case intake:
          case shooter: {
            setLEDStripColor(Constants.LEDs.Colors.passing);
            break;
          }
          case noNote:{
            setLEDIntakingColorChase(Constants.LEDs.Colors.passing);
            break;
          }
        }
        break;
      }

      case Auto:
      break;
    }
  }

  public void setLEDAimingMode(){
    if (isOnTarget){
      switch (noteLocation){
          case shooter: {
            setLEDStripColor(Constants.LEDs.Colors.onTarget);
            break;
          }
          case intake:
          case noNote:{
            setLEDNoNote(Constants.LEDs.Colors.onTarget);
            break;
          }
        }
    } else {
      switch (noteLocation){
          case shooter: {
            setLEDStripColor(Constants.LEDs.Colors.aiming);
            break;
          }
          case intake:
          case noNote:{
            setLEDNoNote(Constants.LEDs.Colors.aiming);
            break;
          }
        }
    } 
  }

  public void setLEDShootingMode(){
    setLEDStripColor(Constants.LEDs.Colors.shooting);
  }
}

