// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  TalonFX intakeMotor = new TalonFX(Constants.IntakeConstants.intakeMotor);
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    TalonFXConfigurator talonFXConfigurator = intakeMotor.getConfigurator();
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    var CurrentLimits = config.CurrentLimits;
    CurrentLimits.StatorCurrentLimit = Constants.IntakeConstants.STATOR_CURRENT_LIMIT2;
    CurrentLimits.SupplyCurrentLimit = Constants.IntakeConstants.CURRENT_LIMIT2;
    CurrentLimits.StatorCurrentLimitEnable = Constants.IntakeConstants.ENABLE_STATOR_CURRENT_LIMIT2;
    CurrentLimits.SupplyCurrentLimitEnable = Constants.IntakeConstants.ENABLE_CURRENT_LIMIT2;

    talonFXConfigurator.apply(config);
    
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
