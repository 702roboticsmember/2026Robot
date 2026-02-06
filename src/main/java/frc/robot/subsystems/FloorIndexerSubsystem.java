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

public class FloorIndexerSubsystem extends SubsystemBase {
  TalonFX FloorIndexMotor = new TalonFX(Constants.IndexerConstants.indexMotorFeeder);
  /** Creates a new FloorIndexerSubsystem. */
  public FloorIndexerSubsystem() {
    
    TalonFXConfigurator talonFXConfigurator = FloorIndexMotor.getConfigurator();
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    var CurrentLimits = config.CurrentLimits;
    CurrentLimits.StatorCurrentLimit = Constants.IndexerConstants.FLOOR_STATOR_CURRENT_LIMIT;
    CurrentLimits.SupplyCurrentLimit = Constants.IndexerConstants.FLOOR_CURRENT_LIMIT;
    CurrentLimits.StatorCurrentLimitEnable = Constants.IndexerConstants.FLOOR_ENABLE_STATOR_CURRENT_LIMIT;
    CurrentLimits.SupplyCurrentLimitEnable = Constants.IndexerConstants.FLOOR_ENABLE_CURRENT_LIMIT;
    
    talonFXConfigurator.apply(config);

  }
  public void setFloorIndexSpeed(double speed) {
    FloorIndexMotor.set(speed);
  }
  public Command spin(DoubleSupplier speed) {
    return Commands.runEnd(
      () -> { setFloorIndexSpeed(speed.getAsDouble()); },
      () -> { setFloorIndexSpeed(0); },
      this
    );
  }

  public double tickToDeg(double ticks) {
    return ticks*Constants.IndexerConstants.gearingMultiplier;
  }
  public double degToTick(double deg) {
    return deg/Constants.IndexerConstants.gearingMultiplier;
  }
  public double getIndexerPos() {
    return tickToDeg(FloorIndexMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
