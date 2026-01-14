// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX motor1 = new TalonFX(Constants.ShooterConstants.shooterMotor1);
  private TalonFX motor2 = new TalonFX(Constants.ShooterConstants.shooterMotor2);
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    motor2.setControl(new Follower(Constants.ShooterConstants.shooterMotor1, MotorAlignmentValue.Aligned));
  }

  public void setSpeed(double speed) {
    motor1.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
