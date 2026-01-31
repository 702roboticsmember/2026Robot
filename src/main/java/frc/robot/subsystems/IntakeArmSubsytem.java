// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeArmSubsytem extends SubsystemBase {
  TalonFX armMotor = new TalonFX(Constants.IntakeConstants.armMotor);
  /** Creates a new IntakeArmSubsytem. */
  public IntakeArmSubsytem() {
        TalonFXConfigurator talonFXConfigurator = armMotor.getConfigurator();
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    var CurrentLimits = config.CurrentLimits;
    CurrentLimits.StatorCurrentLimit = Constants.IntakeConstants.STATOR_CURRENT_LIMIT;
    CurrentLimits.SupplyCurrentLimit = Constants.IntakeConstants.CURRENT_LIMIT;
    CurrentLimits.StatorCurrentLimitEnable = Constants.IntakeConstants.ENABLE_STATOR_CURRENT_LIMIT;
    CurrentLimits.SupplyCurrentLimitEnable = Constants.IntakeConstants.ENABLE_CURRENT_LIMIT;

    var SoftLimits = config.SoftwareLimitSwitch;
    SoftLimits.ForwardSoftLimitEnable = Constants.IntakeConstants.softLimitEnable;
    SoftLimits.ForwardSoftLimitThreshold = degToTick(Constants.IntakeConstants.forwardLimit);
    SoftLimits.ReverseSoftLimitEnable = Constants.IntakeConstants.softLimitEnable;
    SoftLimits.ReverseSoftLimitThreshold = degToTick(Constants.IntakeConstants.reverseLimit);

    talonFXConfigurator.apply(config);
  }

  public void setArmSpeed(double speed){
    armMotor.set(speed);
  }
  public double tickToDeg(double tick){
    return tick * 1/1;
  }
  public double degToTick(double deg){
    return deg * 1/1;
  }
  public double getArmAngle(){
    return tickToDeg(armMotor.getPosition().getValueAsDouble());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
