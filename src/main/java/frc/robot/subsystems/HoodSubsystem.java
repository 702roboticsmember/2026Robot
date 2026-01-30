// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HoodSubsystem extends SubsystemBase {
  TalonFX HoodMotor = new TalonFX(Constants.HoodConstants.HoodMotor);
  /** Creates a new HoodSubsystem. */
  public void setSpeed(double speed) {
    HoodMotor.set(speed);
  }
  public double tickToDeg(double tick){
    return tick * 1/1;
  }
  public double getHoodAngle(){
    return tickToDeg(HoodMotor.getPosition().getValueAsDouble());
  }

  public HoodSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }                               

  public Command spin(DoubleSupplier speed) {
    return Commands.runEnd(() -> {
      setSpeed(speed.getAsDouble());
    }, () -> {
      setSpeed(0);
    }, this);
  }
}
