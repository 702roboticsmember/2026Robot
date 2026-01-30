// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeArmSubsytem extends SubsystemBase {
  TalonFX armMotor = new TalonFX(Constants.IntakeConstants.armMotor);
  /** Creates a new IntakeArmSubsytem. */
  public IntakeArmSubsytem() {}

  public void setArmSpeed(double speed){
    armMotor.set(speed);
  }
  public double tickToDeg(double tick){
    return tick * 1/1;
  }
  public double getArmAngle(){
    return tickToDeg(armMotor.getPosition().getValueAsDouble());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
