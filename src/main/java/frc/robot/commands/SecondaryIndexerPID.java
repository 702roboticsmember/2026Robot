// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SecondaryIndexerPID extends Command {
  /** Creates a new SecondaryIndexerPID. */
  private IndexerSubsystem ind_subsystem;
  private PIDController controller = new PIDController(0,0,0);
  private double target;
  
  public SecondaryIndexerPID(IndexerSubsystem ind_subsystem, double target) {
    this.ind_subsystem = ind_subsystem;
    this.target = target;
    addRequirements(ind_subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setSetpoint(target);
    controller.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controller.calculate(ind_subsystem.getSecondaryDeg());
    ind_subsystem.setSpeedSecondary(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
