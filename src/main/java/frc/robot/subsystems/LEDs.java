// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.leds.Color;
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

  private int intakeFrameCounter = 0;
  private int intakeFrameTimer = 0;

  private int colorChaseFrameCounter = 0;
  private int colorChaseFrameTimer = 0;

  private boolean noNoteColorFrame = true;
  private int noNoteFrameTimer = 0;

  private boolean repeatLEDCheck = true;

  public LEDs(MyRobotState robotState) {
    m_led = new AddressableLED(0); //~
    m_ledBuffer = new AddressableLEDBuffer(81); //~ Copied from Stingray's code
    m_led.setLength(m_ledBuffer.getLength()); //~

    m_led.setData(m_ledBuffer);
    m_led.start();

    mRobotState = robotState;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("LED State (in LEDs)", mLEDMode.toString());

    if (mLEDMode == LEDModeState.intaking && noteLocation != RobotNoteLocation.noNote ) {
      mRobotState.setLEDMode(LEDModeState.idle);
    }

    if (mLEDMode != mRobotState.getLEDMode() || mRobotMode != mRobotState.getRobotMode() || noteLocation != mRobotState.getNoteLocation()
    ||  isOnTarget != mRobotState.isOnTarget() || repeatLEDCheck){
      
      noteLocation = mRobotState.getNoteLocation();
      isOnTarget = mRobotState.isOnTarget();
      mLEDMode = mRobotState.getLEDMode();
      mRobotMode = mRobotState.getRobotMode();
      repeatLEDCheck = false;
      // hasNoteInIntake
      
      switch (mRobotState.getLEDMode()) {
        case idle: {
          setLEDIdleMode();
          break;
        }
        case intaking: {
          setLEDIntakingMode();
          break;
        }
        case aiming: {
          setLEDAimingMode();
          break;
        }
        case shooting: {
          setLEDShootingMode();
          break;
        }    
      }
    }
    
    // This method will be called once per scheduler run
  }

  public void setSingleLEDColor(int pixel, Color color) {
      m_ledBuffer.setRGB(pixel, color.r(), color.g(), color.b());
  }

  public void setLEDIntakingColorChase(Color color){
    if (intakeFrameTimer == 5) {
      int fractionedLEDLength = (m_ledBuffer.getLength() / Constants.LEDs.numberOfIntakingChasers);
      for (int i = 0; i < (Constants.LEDs.numberOfIntakingChasers); i++){
        for (int j = 0; j < (fractionedLEDLength); j++){
          int pixel = j + (fractionedLEDLength * i) + intakeFrameCounter;
          if (pixel >= fractionedLEDLength * (i + 1)){
            pixel = pixel - fractionedLEDLength;
          }
          m_ledBuffer.setRGB(pixel, color.r() / ((fractionedLEDLength - j) * (fractionedLEDLength - j) * (((fractionedLEDLength - j) / 2) + 1)), color.g() / ((fractionedLEDLength - j) * (fractionedLEDLength - j)), color.b() / ((fractionedLEDLength - j) * (fractionedLEDLength - j)));
        }
      }
      intakeFrameCounter ++;
      if (intakeFrameCounter >= fractionedLEDLength) {
        intakeFrameCounter = 0;
      }
      m_led.setData(m_ledBuffer);
      intakeFrameTimer = 0;
    }
    repeatLEDCheck = true;
    intakeFrameTimer++;
  }

  public void setLEDStripColor(Color color) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        setSingleLEDColor(i, color);
    }

    m_led.setData(m_ledBuffer);
  }

   public void setLEDTwoColorChase(Color color1, Color color2){
    if (colorChaseFrameTimer == 3) {
      int fractionedLEDLength = (m_ledBuffer.getLength() / Constants.LEDs.numberOfChaserChasers);
      for (int i = 0; i < (Constants.LEDs.numberOfChaserChasers); i++){
        for (int j = 0; j < (fractionedLEDLength); j++){
          int pixel = j + (fractionedLEDLength * i) + colorChaseFrameCounter;
          if (pixel >= fractionedLEDLength * (i + 1)){
            pixel = pixel - fractionedLEDLength;
          }
          if (j >= fractionedLEDLength / 2){
             m_ledBuffer.setRGB(pixel, color1.r() / (fractionedLEDLength - j), color1.b() / (fractionedLEDLength - j), color1.r() / (fractionedLEDLength - j));
          } else{
             m_ledBuffer.setRGB(pixel, color2.r() / (fractionedLEDLength - j), color2.g() / (fractionedLEDLength - j), color2.b() / (fractionedLEDLength - j));
          }
        }
      }
      colorChaseFrameCounter ++;
      if (colorChaseFrameCounter >= fractionedLEDLength) {
        colorChaseFrameCounter = 0;
      }
      for (int k = (fractionedLEDLength * Constants.LEDs.numberOfChaserChasers); k < m_ledBuffer.getLength(); k++){
          m_ledBuffer.setRGB(k, color2.r(), color2.g(), color2.b());
      }
      m_led.setData(m_ledBuffer);
      colorChaseFrameTimer = 0;
    }
    repeatLEDCheck = true;
    colorChaseFrameTimer++;
  }


  public void setLEDNoNote(Color color) {
    if (noNoteFrameTimer == 10){
      if (noNoteColorFrame){
        setLEDStripColor(color);
        noNoteColorFrame = false;
      } else if (mLEDMode != LEDModeState.aiming){
        setLEDStripColor(Constants.LEDs.Colors.white);
        noNoteColorFrame = true;
      } else {
        setLEDStripColor(new Color(0, 0, 0));
        noNoteColorFrame = true;
      }
      noNoteFrameTimer = 0;
    }
  noNoteFrameTimer ++;
  repeatLEDCheck = true;
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

      case Auto: {
        setLEDTwoColorChase(Constants.LEDs.Colors.white, Constants.LEDs.Colors.blue);
        break;
      }
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

      case Auto: {
        setLEDTwoColorChase(Constants.LEDs.Colors.white, Constants.LEDs.Colors.blue);
        break;
      }
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

