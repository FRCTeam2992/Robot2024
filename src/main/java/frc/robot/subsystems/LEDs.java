// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MyRobotState;
import frc.robot.MyRobotState.LEDModeState;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;

  public boolean hasNoteInIntake = false;
  public boolean hasNoteInShooter = false;
  public boolean isOnTarget = false;

  public LEDModeState mLEDMode = LEDModeState.idle;

  private Intake mIntake;
  private Feeder mFeeder;
  private Shooter mShooter;
  private Elevator mElevator;
  private ShooterPivot mPivot;
  private MyRobotState mRobotState;

  public LEDs(Intake intake, Feeder feeder, Shooter shooter, Elevator elevator, ShooterPivot pivot, MyRobotState robotState) {
    m_led = new AddressableLED(0); //~
    m_ledBuffer = new AddressableLEDBuffer(17); //~ Copied from Stingray's code
    m_led.setLength(m_ledBuffer.getLength()); //~

    m_led.setData(m_ledBuffer);
    m_led.start();

    mIntake = intake;
    mFeeder = feeder;
    mShooter = shooter;
    mElevator = elevator;
    mPivot = pivot;
    mRobotState = robotState;
  }

  @Override
  public void periodic() {
    
    if (mLEDMode != mRobotState.getLEDMode() || hasNoteInShooter != mFeeder.getBeamBreakTriggered() 
    ||  isOnTarget != (mElevator.atPosition() && mPivot.atTarget() && mShooter.atShooterRPM())){
      
      hasNoteInShooter = mFeeder.getBeamBreakTriggered();
      isOnTarget = (mElevator.atPosition() && mPivot.atTarget() && mShooter.atShooterRPM());
      mLEDMode = mRobotState.getLEDMode();
      
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


  public void setLEDStripColor(Color color) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        setSingleLEDColor(i, color);
    }

    m_led.setData(m_ledBuffer);
  }
}

