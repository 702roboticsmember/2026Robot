// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
// import frc.robot.subsystems.Subsystem;
import frc.robot.subsystems.TurretSubsytem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretRotatePIDCommand extends Command {
  private TurretSubsytem turretSubsystem;
  private PIDController PID = new PIDController(Constants.TurretConstants.kP, Constants.TurretConstants.kI, Constants.TurretConstants.kD);
  private double target;
  /** Creates a new TurretRotatePIDCommand. */
  TurretRotatePIDCommand(TurretSubsytem subsystem, double target) {
    turretSubsystem = subsystem;
    addRequirements(subsystem);
    this.target = target;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PID.setSetpoint(target);
    PID.setTolerance(Constants.TurretConstants.Tolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = PID.calculate(turretSubsystem.getArmAngle());
    turretSubsystem.setAngleSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turretSubsystem.setAngleSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return PID.atSetpoint();
  }
}
