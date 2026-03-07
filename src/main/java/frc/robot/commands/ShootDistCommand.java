// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Field;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpersCameronEdition;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootDistCommand extends Command {
 
  private TurretSubsystem t_TurretSubsystem ;
  private HoodSubsystem h_HoodSubsystem;
  private ShooterSubsystem s_ShooterSubsystem;
  

  private double g = Constants.PhysicsConstants.gravity;
  private double h = Constants.PhysicsConstants.HubHeight;
  private double i = Constants.PhysicsConstants.BallIntialHeight;
  private double dist;



 


  

  
  /** Creates a new AutoAimCommand. */
  public ShootDistCommand(double dist, TurretSubsystem t_TurretSubsystem, HoodSubsystem h_HoodSubsystem, ShooterSubsystem s_ShooterSubsystem) {
    this.t_TurretSubsystem = t_TurretSubsystem;
    this.s_ShooterSubsystem = s_ShooterSubsystem;
    this.h_HoodSubsystem = h_HoodSubsystem;
    addRequirements(t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem);
    this.dist = dist;
   
    // Use addRequirements() here to declare subsystem dependencies.
  }

 

  
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    
    
    
    //limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.limelightConstants.limelightTurret);

   

   
    
    // Stationary shoot code
    // double RobotBasedAngle = getTurretAngleToHub(pose).getDegrees();
    // double distance = getDistance(pose);
    // double vy = CalculateVy(distance);
    // double vx = CalculateVx(distance, vy);
    // double vs = CalculateVs(vx, vy, 0, 0);
    // double shootAngle = CalculateShootAngle(vx, vy, 0, 0);

    // Shoot on the move code
    double Dx = dist;
    double vy = CalculateVy(Dx);
   
    //double t = timeTillTarget(vy);
   
    double distance = CalculateShotDistance(0, Dx);
    double vx = CalculateVx(distance, vy);
    double vs = CalculateVs(vx, vy, 0, 0);
    double shootAngle = CalculateShootAngle(vx, vy, 0, 0);
   
   
   
    
    
    // SmartDashboard.putNumber("poseturretangle2", pose.getRotation().getDegrees());
    SmartDashboard.putNumber("poseturretdist", distance);
    // SmartDashboard.putNumber("poitx", poi.getX());
    // SmartDashboard.putNumber("poity", poi.getY());
    
   
    
      //Linear regression.
      double output = vs * 2.3492 + 0.26157;//TODO Calibrate based on input velocity vs ball actual velocity
      //2.2492 + 0.56157
      //Quartic regression


      //double output = 0.00918838 * Math.pow(vs, 4) - 0.199587 * Math.pow(vs, 3) + 1.45776*Math.pow(vs, 2) - 2.20965* vs+ 5.3498;
      SmartDashboard.putNumber("output", output);
      s_ShooterSubsystem.setVelocity(output);
      h_HoodSubsystem.goToAngle(shootAngle);
    
    //s_ShooterSubsystem.setVelocity(vs * 2.1492 + 0.56157);

  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
    public boolean isFinished(){
    return false;
  }

 
 

  



/**
   * Calculates the new distance the robot has to shoot based on time it takes for the ball to reach target
   * @param OffsetAngle
   * @param Dx
   * @return the distance the ball must travel.
   */
  public double CalculateShotDistance(double OffsetAngle, double Dx){
    return Dx/Math.cos(Math.toRadians(OffsetAngle));
  }

  /**CalculateVy
   *  @param Dx psudo value that takes in distance from hub and not desired distance of shot.
   * @return Vy (Vertical velocity) based on a preset formula to help reduce complexity.
   */
  public double CalculateVy(double Dx){
    return MathUtil.clamp(0.098438*Dx + 5.81997, 5.8, 1000);//Originally a constant but I found adjusting the Vy and in turn the max height I can increase accuracy from afar.
  }

  public double getBasicVy(){
    return 6.2;
  }

  public double timeTillTarget(double Vy){
    double a = -g * 0.5;
    double b = Vy;
    double c = -(h-i);
    double Input = (b * b) - (4* a * c);
    if(Input < 0){
      Input = 0;//TODO Constant based on data
    }
    return ((-b) - Math.sqrt(Input))/ (2*a);
  }


  public double CalculateVx(double distance, double Vy){
    double a = -g * distance * distance * 0.5;
    double b = Vy*distance;
    double c = -(h - i);
    double Input = (b * b) - 4*(a * c);
    if(Input < 0){
      Input = 0;//TODO Constant based on data
    }

    return (2*(a)) / ((-b) - Math.sqrt(Input));
  }

  public double CalculateVs(double Vx, double Vy, double Vrx, double angleOffset){
    double vrx = Vrx* Math.cos(Math.toRadians(angleOffset));
    double Input = ((Vx - vrx) * (Vx - vrx)) + (Vy * Vy);
    if (Input < 0){
      Input = 0;//TODO Constant based on data
    }
    return Math.sqrt(Input);
  }

  public double CalculateShootAngle(double Vx, double Vy, double Vrx, double angleOffset){
    double vrx = Vrx* Math.cos(Math.toRadians(angleOffset));
    double HoodAngle = Vy/(Vx - vrx);

   
    if(Vy/(Vx - vrx) == (Math.PI/2)){
      return 0;//TODO Constant based on data
    }
   
    else if( Vy/(Vx - vrx) == (-Math.PI/2) ){
      return 0;//TODO Constant based on data
    }
    else{
      return Math.toDegrees(Math.atan(HoodAngle));
    }
  }

  public Rotation2d getTurretAngle(Pose2d pose){
    return pose.getRotation();
  }

  public double getIMUYaw(){
    return LimelightHelpersCameronEdition.getIMUData("limelight").gyroY;
  }
}
