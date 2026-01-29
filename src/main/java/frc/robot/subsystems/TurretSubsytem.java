// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

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
  public double getArmAngle(){
    return tickToDeg(angleMotor.getPosition().getValueAsDouble());
  }

  public TurretSubsytem() {

  
    // TalonFXConfigurator talonFXConfigurator = angleMotor.getConfigurator();
    // CurrentLimitsConfigs limits = new CurrentLimitsConfigs();

    // limits.StatorCurrentLimit = Constants.ClimbConstants.STATOR_CURRENT_LIMIT;
    // limits.SupplyCurrentLimit = Constants.ClimbConstants.CURRENT_LIMIT;
    // limits.StatorCurrentLimitEnable = Constants.ClimbConstants.ENABLE_STATOR_CURRENT_LIMIT;
    // limits.SupplyCurrentLimitEnable = Constants.ClimbConstants.ENABLE_CURRENT_LIMIT;

    // talonFXConfigurator.apply(limits);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}//
