// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  private TalonFX motor = new TalonFX(Constants.ClimbConstants.climbMotor);
  
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    TalonFXConfigurator talonFXConfigurator = motor.getConfigurator();
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    var CurrentLimits = config.CurrentLimits;
    CurrentLimits.StatorCurrentLimit = Constants.ClimbConstants.STATOR_CURRENT_LIMIT;
    CurrentLimits.SupplyCurrentLimit = Constants.ClimbConstants.CURRENT_LIMIT;
    CurrentLimits.StatorCurrentLimitEnable = Constants.ClimbConstants.ENABLE_STATOR_CURRENT_LIMIT;
    CurrentLimits.SupplyCurrentLimitEnable = Constants.ClimbConstants.ENABLE_CURRENT_LIMIT;

    var SoftLimits = config.SoftwareLimitSwitch;
    SoftLimits.ForwardSoftLimitEnable = Constants.ClimbConstants.softLimitEnable;
    SoftLimits.ForwardSoftLimitThreshold = DistToTick(Constants.ClimbConstants.forwardLimit);
    SoftLimits.ReverseSoftLimitEnable = Constants.ClimbConstants.softLimitEnable;
    SoftLimits.ReverseSoftLimitThreshold = DistToTick(Constants.ClimbConstants.reverseLimit);

    talonFXConfigurator.apply(config);
  }

   public double getClimbAngle(){
    return TickToDist(motor.getPosition().getValueAsDouble());
  }

  public double TickToDist(double ticks){
    return ticks *1/1;
  }

  public double DistToTick(double dist){
    return dist * 1/1;
  }

  public void setSpeed(double speed){
    motor.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
