// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  TalonFX intakeMotor = new TalonFX(Constants.IntakeConstants.intakeMotor);
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    TalonFXConfigurator talonFXConfigurator = intakeMotor.getConfigurator();
    CurrentLimitsConfigs limits = new CurrentLimitsConfigs();

    limits.StatorCurrentLimit = Constants.IntakeConstants.STATOR_CURRENT_LIMIT;
    limits.SupplyCurrentLimit = Constants.IntakeConstants.CURRENT_LIMIT;
    limits.StatorCurrentLimitEnable = Constants.IntakeConstants.ENABLE_STATOR_CURRENT_LIMIT;
    limits.SupplyCurrentLimitEnable = Constants.IntakeConstants.ENABLE_CURRENT_LIMIT;

    talonFXConfigurator.apply(limits);

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public Command spin(DoubleSupplier speed) {
    return Commands.runEnd(
      () -> { setIntakeSpeed(speed.getAsDouble()); },
      () -> { setIntakeSpeed(0); },
      this
    );
  }
}
