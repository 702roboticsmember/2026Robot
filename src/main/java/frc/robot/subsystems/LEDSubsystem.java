// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  private AddressableLED LED = new AddressableLED(Constants.LEDConstants.LEDPort);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LEDConstants.BufferLength);
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    LED.start();
    
  }

  @Override
  public void periodic() {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, Color.kRed);
    }
    
    
    
    LED.setData(buffer);
    // This method will be called once per scheduler run
  }
}
