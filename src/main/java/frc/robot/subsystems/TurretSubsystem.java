// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;



public class TurretSubsystem extends SubsystemBase {
  private TalonFX Motor = new TalonFX(Constants.TurretConstants.TurretMotorID);
  private MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
  /** Creates a new ClimbSubsystem. */
   public TurretSubsystem() { 
    //m_encoderFR.setSimDevice(SimDevice.create("encoder"));
    TalonFXConfiguration turretConfig = new TalonFXConfiguration();
      turretConfig.CurrentLimits.StatorCurrentLimit = Constants.TurretConstants.CURRENT_LIMIT;
            turretConfig.CurrentLimits.SupplyCurrentLimit = Constants.TurretConstants.CURRENT_LIMIT;
            turretConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.TurretConstants.ENABLE_CURRENT_LIMIT;
            turretConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.TurretConstants.ENABLE_CURRENT_LIMIT;
            //turretConfig.
            

            turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = Constants.TurretConstants.LimitEnable;
            turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = getDegreesToticks(Constants.TurretConstants.forwardLimit);
            turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = Constants.TurretConstants.LimitEnable;
            turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = getDegreesToticks(Constants.TurretConstants.reverseLimit);

            var s_slot0Configs = turretConfig.Slot0;
              s_slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
              s_slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
              s_slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
              s_slot0Configs.kP = 7.5; // An error of 1 rps results in 0.11 V output
              s_slot0Configs.kI = 0; // no output for integrated error
              s_slot0Configs.kD = 0.1; // no output for error derivative
    
            var motionMagicConfigs = turretConfig.MotionMagic;
              motionMagicConfigs.MotionMagicCruiseVelocity = 40; // Target cruise velocity of 80 rps
              motionMagicConfigs.MotionMagicAcceleration = 80; // Target acceleration of 160 rps/s (0.5 seconds)
              motionMagicConfigs.MotionMagicJerk = 800; // Target jerk of 1600 rps/s/s (0.1 seconds)

      Motor.getConfigurator().apply(turretConfig);
    
  // /** Creates a new ReleaseSubsystem. */
 }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("turretangle", getAngleAsDouble());
    
    SmartDashboard.putNumber("limelightrobotyaw", getLimelightYaw());
    SmartDashboard.putNumber("turretAngleTicks", getAngleAsTicks());

   
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    Motor.set(speed);
    SmartDashboard.putBoolean("hiiiiiii", true);
    
    
  }

  public Command run(DoubleSupplier input){
    
    return this.runEnd(() -> this.setSpeed(input.getAsDouble()), () -> this.setSpeed(0.0));
  }

  public Rotation2d getAngle() {
    return new Rotation2d(getAngleAsDouble());
    
    
  }

  public double getAngleAsDouble() {
    return getTicksToDegrees(Motor.getPosition().getValueAsDouble());
  }
  public double getAngleAsTicks() {
    return Motor.getPosition().getValueAsDouble();
  }

  public double getTicksToDegrees(double ticks){
    return ticks * 360/Constants.TurretConstants.TurretConversionRate;
  }

 public double getDegreesToticks(double degrees){
    return degrees * Constants.TurretConstants.TurretConversionRate/360;
  }

  public void setAngle(double degrees){
    Motor.setPosition(getDegreesToticks(degrees));
  }

  public double getVel(){
    return Motor.getVelocity().getValueAsDouble();
  }
  public void setVoltage(double Voltage){
    Motor.setVoltage(Voltage);
  }

  
  public double getLimelightYaw(){
    double limelightMeasurement = LimelightHelpers.getIMUData("limelight").robotYaw;
    
    return limelightMeasurement;
  }

  public void goToAngle(double angle){
    Motor.setControl(motionMagic.withPosition(getDegreesToticks(angle)));
  }

  public void goToAngleOffset(double angleOffset){
  
    Motor.setControl(motionMagic.withPosition(getDegreesToticks(getAngleAsDouble() + angleOffset)));
  
  }

}