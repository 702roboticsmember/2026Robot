// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX FlywheelMotor1 = new TalonFX(Constants.ShooterConstants.shooterMotor1);
  private TalonFX FlywheelMotor2 = new TalonFX(Constants.ShooterConstants.shooterMotor2);

  private MotionMagicVelocityDutyCycle velControl = new MotionMagicVelocityDutyCycle(0);
  //private TalonFX TurretMotor = new TalonFX(Constants.ShooterConstants.angleMotor);
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = Constants.ShooterConstants.kS; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = Constants.ShooterConstants.kV; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = Constants.ShooterConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = Constants.ShooterConstants.kP; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = Constants.ShooterConstants.kI; // no output for integrated error
    slot0Configs.kD = Constants.ShooterConstants.kD; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.ShooterConstants.CruiseVelocity; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = Constants.ShooterConstants.Acceleration; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = Constants.ShooterConstants.Jerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

        TalonFXConfigurator talonFXConfigurator = FlywheelMotor1.getConfigurator();
        TalonFXConfigurator talonFXConfigurator2 = FlywheelMotor2.getConfigurator();
    CurrentLimitsConfigs limits = new CurrentLimitsConfigs();

    limits.StatorCurrentLimit = Constants.ShooterConstants.STATOR_CURRENT_LIMIT;
    limits.SupplyCurrentLimit = Constants.ShooterConstants.CURRENT_LIMIT;
    limits.StatorCurrentLimitEnable = Constants.ShooterConstants.ENABLE_STATOR_CURRENT_LIMIT;
    limits.SupplyCurrentLimitEnable = Constants.ShooterConstants.ENABLE_CURRENT_LIMIT;

    talonFXConfigurator.apply(limits);
    talonFXConfigurator2.apply(limits);


    FlywheelMotor1.getConfigurator().apply(talonFXConfigs);
    FlywheelMotor2.getConfigurator().apply(talonFXConfigs);
    FlywheelMotor2.setControl(new Follower(Constants.ShooterConstants.shooterMotor1, MotorAlignmentValue.Aligned));
    
  }

  public void setSpeed(double speed) {
    FlywheelMotor1.set(speed);
  }
   //public double tickToDeg("")

  public void setVelocity(double velocity){
    FlywheelMotor1.setControl(velControl.withVelocity(velocity));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command spin(DoubleSupplier speed) {
    return Commands.runEnd(() -> {
      setSpeed(speed.getAsDouble());
    }, () -> {
      setSpeed(0);
    }, this);
  }
}
