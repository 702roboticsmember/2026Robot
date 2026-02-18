// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAimCommand extends Command {
  private String limelightName = Constants.limelightConstants.limelightTurret;
  private double txOffset;
 
  private TurretSubsystem t_TurretSubsystem ;
  private HoodSubsystem h_HoodSubsystem;
  private ShooterSubsystem s_ShooterSubsystem;
  

  private double g = Constants.PhysicsConstants.gravity;
  private double h = Constants.PhysicsConstants.HubHeight;
  private double i = Constants.PhysicsConstants.BallIntialHeight;

  private Translation2d poi;
  public LimelightHelpers.PoseEstimate limelightMeasurement;
  private BooleanSupplier hoodUp;
 
  boolean angleRight;
  public void checkAngle (){
   Pose2d pose = Constants.TurretConstants.turretPose2d;
    double off = getAngleToHub(pose).getTan()*getDistance(pose);
   if (off < 2 && off > -2){
     this.angleRight = true;
    }else{
     this.angleRight = false;
    }
  }

  
  /** Creates a new AutoAimCommand. */
  public AutoAimCommand(Translation2d poi, TurretSubsystem t_TurretSubsystem, HoodSubsystem h_HoodSubsystem, ShooterSubsystem s_ShooterSubsystem, BooleanSupplier hoodUp) {
    this.t_TurretSubsystem = t_TurretSubsystem;
    this.s_ShooterSubsystem = s_ShooterSubsystem;
    this.h_HoodSubsystem = h_HoodSubsystem;
    addRequirements(t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem);
    this.poi = poi;
    this.hoodUp = hoodUp;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    Pose2d pose = Constants.TurretConstants.turretPose2d;
    limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    
    
    double turretangle = getTurretAngleToHub(pose).getDegrees();
    double distance = getDistance(pose);
    double vy = CalculateVy(distance);
    double vx = CalculateVx(distance, vy);
    double vs = CalculateVs(vx, vy, 0);
    double shootAngle = CalculateShootAngle(vx, vy, 0);
    SmartDashboard.putNumber("shootAngle", shootAngle );
    SmartDashboard.putNumber("shootSpeed", vs);
    SmartDashboard.putNumber("shootSpeedx", vx);
    SmartDashboard.putNumber("shootSpeedy", vy);
    SmartDashboard.putNumber("poseturret angle", turretangle);
    SmartDashboard.putNumber("poseturretdist", distance);

    t_TurretSubsystem.goToAngle(-turretangle);
    if(hoodUp.getAsBoolean())h_HoodSubsystem.goToAngle(shootAngle);
    else h_HoodSubsystem.goToAngle(Constants.HoodConstants.forwardLimit);
    s_ShooterSubsystem.setVelocity(vs * 2.35);

  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double getDistance(Pose2d Pose) {
    return Pose.getTranslation().getDistance(poi);
  }

  private Rotation2d getAngleToHub(Pose2d Pose){
    return poi.minus(Pose.getTranslation()).getAngle();
  }

  public Rotation2d getTurretAngleToHub(Pose2d pose){
    return getAngleToHub(pose).minus(pose.getRotation());
  }

  /**
   * getVrx:
   *  unfinished 
   * @return nothing right now.
   */
  public double getVrx(Pose2d pose){
    LimelightHelpers.IMUData data = LimelightHelpers.getIMUData("limelight");
    double gyrox = data.gyroX;
    double gyroy = data.gyroY;
    double gyroz = data.gyroZ;
    double robotYaw = data.robotYaw;
    Rotation2d angleToHub = getAngleToHub(pose);
    double pitch = data.Pitch;
    double roll = data.Roll;
    double yaw = data.Yaw;
    return 0;
  }

  

  /**
   * getVrz:
   *  unfinished 
   * @return nothing right now.
   */
  public double getVrz(Pose2d pose){
    LimelightHelpers.IMUData data = LimelightHelpers.getIMUData("limelight");
    double gyrox = data.gyroX;
    double gyroy = data.gyroY;
    double gyroz = data.gyroZ;
    double robotYaw = data.robotYaw;
    Rotation2d angleToHub = getAngleToHub(pose);
    double pitch = data.Pitch;
    double roll = data.Roll;
    return 0;
  }


  public double CalculateOffset(double Vrz, double Dx, double t){
    return Math.tanh((Vrz* t)/Dx);
  }

  public double CalculateVy(double Dx){
    return 0.098438*Dx + 5.81997;
  }

  public double timeTillTarget(double Vy){
    double a = -g * 0.5;
    double b = Vy;
    double c = -(h-i);
    return (-b) - Math.sqrt((b * b) - (4* a * c));
  }


  public double CalculateVx(double distance, double Vy){
    double a = -g * distance * distance * 0.5;
    double b = Vy*distance;
    double c = -(h - i);
    return (2*(a)) / ((-b) - Math.sqrt((b * b) - 4*(a * c)));
  }

  public double CalculateVs(double Vx, double Vy, double Vrx){
    return Math.sqrt(((Vx - Vrx) * (Vx - Vrx)) + (Vy * Vy));
  }

  public double CalculateShootAngle(double Vx, double Vy, double Vrx){
    return Math.toDegrees(Math.atan(Vy/(Vx - Vrx)));
  }

  public double getVrz(Pose2d pose, double Vz){
    return getTurretAngleToHub(pose).getCos() * Vz;
  }

  public Rotation2d getTurretAngle(Pose2d pose){
    return pose.getRotation();
  }

  public double getIMUYaw(){
    return LimelightHelpers.getIMUData("limelight").gyroY;
  }

  

}
