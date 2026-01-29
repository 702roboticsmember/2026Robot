// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class IndexerSubsystem extends SubsystemBase {

  TalonFX indexMotorPrimary = new TalonFX(Constants.IndexerConstants.indexMotorFeeder);
  TalonFX indexMotorSecondary = new TalonFX(Constants.IndexerConstants.indexMotorTurret);

  public double speed;

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {
    indexMotorPrimary.getConfigurator().apply(Robot.CTRE_CONFIGS.indexConfigs);
    indexMotorSecondary.getConfigurator().apply(Robot.CTRE_CONFIGS.indexConfigs);
  }

  public void setSpeedSecondary(double speed){
    indexMotorSecondary.set(speed);
  }

 public void setSpeedPrimary(double speed){
    indexMotorPrimary.set(speed);
 }
 public double getPrimaryDeg(){
  return primaryTickToDeg(indexMotorPrimary.getPosition().getValueAsDouble());
 }
  public double getSecondaryDeg(){
    return secondaryTickToDeg(indexMotorSecondary.getPosition().getValueAsDouble());
  }
 
  public double secondaryTickToDeg(double Tick){
    return Tick * 1/1;
}
 public double primaryTickToDeg( double tick){
  return tick * 1/1;
 }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
