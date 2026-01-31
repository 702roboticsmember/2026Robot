// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class IndexerSubsystem extends SubsystemBase {

  TalonFX indexMotorPrimary = new TalonFX(Constants.IndexerConstants.indexMotorFeeder);
  TalonFX indexMotorSecondary = new TalonFX(Constants.IndexerConstants.indexMotorTurret);

  public double speed;

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {
    //Primary
    TalonFXConfigurator talonFXConfigurator = indexMotorPrimary.getConfigurator();
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    var CurrentLimits = config.CurrentLimits;
    CurrentLimits.StatorCurrentLimit = Constants.IndexerConstants.STATOR_CURRENT_LIMIT;
    CurrentLimits.SupplyCurrentLimit = Constants.IndexerConstants.CURRENT_LIMIT;
    CurrentLimits.StatorCurrentLimitEnable = Constants.IndexerConstants.ENABLE_STATOR_CURRENT_LIMIT;
    CurrentLimits.SupplyCurrentLimitEnable = Constants.IndexerConstants.ENABLE_CURRENT_LIMIT;

    //Secondary
            TalonFXConfigurator talonFXConfiguratorSecondary = indexMotorSecondary.getConfigurator();
    TalonFXConfiguration configSecondary = new TalonFXConfiguration();
    
    var CurrentLimitsSecondary = config.CurrentLimits;
    CurrentLimitsSecondary.StatorCurrentLimit = Constants.IndexerConstants.STATOR_CURRENT_LIMIT_SECONDARY;
    CurrentLimitsSecondary.SupplyCurrentLimit = Constants.IndexerConstants.CURRENT_LIMIT_SECONDARY;
    CurrentLimitsSecondary.StatorCurrentLimitEnable = Constants.IndexerConstants.ENABLE_STATOR_CURRENT_LIMIT_SECONDARY;
    CurrentLimitsSecondary.SupplyCurrentLimitEnable = Constants.IndexerConstants.ENABLE_CURRENT_LIMIT_SECONDARY;
    //apply
    talonFXConfigurator.apply(config);
    talonFXConfiguratorSecondary.apply(configSecondary);
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

  public Command spin(DoubleSupplier speedPrimary, DoubleSupplier speedSecondary) {
    return Commands.runEnd(() -> {
      setSpeedPrimary(speedPrimary.getAsDouble()); //Constants.IndexerConstants.PrimarySpeed
      setSpeedSecondary(speedSecondary.getAsDouble()); //Constants.IndexerConstants.SecondarySpeed
    }, () -> {
      setSpeedPrimary(0);
      setSpeedSecondary(0);
    }, this);
  }
}
