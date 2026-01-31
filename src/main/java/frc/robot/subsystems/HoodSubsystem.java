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

public class HoodSubsystem extends SubsystemBase {
  TalonFX HoodMotor = new TalonFX(Constants.HoodConstants.HoodMotor);
  /** Creates a new HoodSubsystem. */
 

  public HoodSubsystem() {
        TalonFXConfigurator talonFXConfigurator = HoodMotor.getConfigurator();
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    var CurrentLimits = config.CurrentLimits;
    CurrentLimits.StatorCurrentLimit = Constants.HoodConstants.STATOR_CURRENT_LIMIT;
    CurrentLimits.SupplyCurrentLimit = Constants.HoodConstants.CURRENT_LIMIT;
    CurrentLimits.StatorCurrentLimitEnable = Constants.HoodConstants.ENABLE_STATOR_CURRENT_LIMIT;
    CurrentLimits.SupplyCurrentLimitEnable = Constants.HoodConstants.ENABLE_CURRENT_LIMIT;

    var SoftLimits = config.SoftwareLimitSwitch;
    SoftLimits.ForwardSoftLimitEnable = Constants.HoodConstants.softLimitEnable;
    SoftLimits.ForwardSoftLimitThreshold = degToTick(Constants.HoodConstants.forwardLimit);
    SoftLimits.ReverseSoftLimitEnable = Constants.HoodConstants.softLimitEnable;
    SoftLimits.ReverseSoftLimitThreshold = degToTick(Constants.HoodConstants.reverseLimit);

    talonFXConfigurator.apply(config);
  }

   public void setSpeed(double speed) {
    HoodMotor.set(speed);
  }
  public double tickToDeg(double tick){
    return tick * 1/1;
  }
  public double degToTick(double deg){
    return deg * 1/1;
  }
  public double getHoodAngle(){
    return tickToDeg(HoodMotor.getPosition().getValueAsDouble());
  }                  

  public Command spin(DoubleSupplier speed) {
    return Commands.runEnd(() -> {
      setSpeed(speed.getAsDouble());
    }, () -> {
      setSpeed(0);
    }, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }    
}
