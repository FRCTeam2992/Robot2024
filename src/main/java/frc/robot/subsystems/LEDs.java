// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;

  public LEDs() {
    m_led = new AddressableLED(0); //~
    m_ledBuffer = new AddressableLEDBuffer(17); //~ Copied from Stingray's code
    m_led.setLength(m_ledBuffer.getLength()); //~

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
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

