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
//9800 is better
public class TurretSubsytem extends SubsystemBase {
  public TalonFX angleMotor = new TalonFX(Constants.TurretConstants.angleMotor);
  /** Creates a new TurretSubsytem. */
  public void setAngleSpeed(double speed){
    angleMotor.set(speed);
  }
  public double tickToDeg(double tick){ 
    return tick * 1/1;
  }
  public double degToTick(double deg){
    return deg *1/1;
  }

 
  public double getAngle(){
    return tickToDeg(angleMotor.getPosition().getValueAsDouble());
  }

  public TurretSubsytem() {

    TalonFXConfigurator talonFXConfigurator = angleMotor.getConfigurator();
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    var CurrentLimits = config.CurrentLimits;
    CurrentLimits.StatorCurrentLimit = Constants.TurretConstants.STATOR_CURRENT_LIMIT;
    CurrentLimits.SupplyCurrentLimit = Constants.TurretConstants.CURRENT_LIMIT;
    CurrentLimits.StatorCurrentLimitEnable = Constants.TurretConstants.ENABLE_STATOR_CURRENT_LIMIT;
    CurrentLimits.SupplyCurrentLimitEnable = Constants.TurretConstants.ENABLE_CURRENT_LIMIT;

    var SoftLimits = config.SoftwareLimitSwitch;
    SoftLimits.ForwardSoftLimitEnable = Constants.TurretConstants.softLimitEnable;
    SoftLimits.ForwardSoftLimitThreshold = degToTick(Constants.TurretConstants.forwardLimit);
    SoftLimits.ReverseSoftLimitEnable = Constants.TurretConstants.softLimitEnable;
    SoftLimits.ReverseSoftLimitThreshold = degToTick(Constants.TurretConstants.reverseLimit);

    talonFXConfigurator.apply(config);
  
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command spin(DoubleSupplier speed) {
    return Commands.runEnd(() -> {
      setAngleSpeed(speed.getAsDouble());
    }, () -> {
      setAngleSpeed(0);
    }, this);
  }
}//
