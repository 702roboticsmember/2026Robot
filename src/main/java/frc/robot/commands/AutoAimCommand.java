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
import frc.robot.RobotContainer;
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

  public AutoAimCommand(TurretSubsystem t_TurretSubsystem, HoodSubsystem h_HoodSubsystem, ShooterSubsystem s_ShooterSubsystem, BooleanSupplier hoodUp) {
    this.t_TurretSubsystem = t_TurretSubsystem;
    this.s_ShooterSubsystem = s_ShooterSubsystem;
    this.h_HoodSubsystem = h_HoodSubsystem;
    addRequirements(t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem);
    this.poi = RobotContainer.currentPOI.location;
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
    this.poi = RobotContainer.currentPOI.location;
    
    
    Pose2d Robotpose = Constants.Swerve.swervePoseEstimator.getEstimatedPosition();
    Pose2d pose = RobotPoseAdjustedTolimelightTurret(Robotpose);
    //limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.limelightConstants.limelightTurret);

    checkAngle();
    SmartDashboard.putBoolean("good to shoot", angleRight);
    
    
    double turretangle = getTurretAngleToHub(pose).getDegrees();
    double RobotBasedAngle = getTurretAngleToHub(RobotPoseAdjustedTolimelightTurret(Robotpose)).getDegrees();
    double distance = getDistance(pose);
    double vy = CalculateVy(distance);
    double vx = CalculateVx(distance, vy);
    double vs = CalculateVs(vx, vy, 0);
    double shootAngle = CalculateShootAngle(vx, vy, 0);
    // if(turretangle > Constants.TurretConstants.forwardLimit){
    //   turretangle =-360;
    // }
    // if(turretangle < Constants.TurretConstants.reverseLimit){
    //   turretangle =+360;
    // }
    if(RobotBasedAngle > Constants.TurretConstants.forwardLimit){
      RobotBasedAngle -=360;
    }
    if(RobotBasedAngle < Constants.TurretConstants.reverseLimit){
      RobotBasedAngle +=360;
    }
    SmartDashboard.putNumber("shootAngle", shootAngle );
    SmartDashboard.putNumber("heading", Robotpose.getRotation().getDegrees());
    SmartDashboard.putNumber("posex", Robotpose.getX());
    SmartDashboard.putNumber("posey", Robotpose.getY());
    SmartDashboard.putNumber("odtes", RobotBasedAngle);
    SmartDashboard.putNumber("shootSpeed", vs);
    SmartDashboard.putNumber("shootSpeedx", vx);
    SmartDashboard.putNumber("shootSpeedy", vy);
    SmartDashboard.putNumber("poseturret angle", turretangle);
    SmartDashboard.putNumber("poseturretangle2", pose.getRotation().getDegrees());
    SmartDashboard.putNumber("poseturretdist", distance);
    SmartDashboard.putNumber("poitx", poi.getX());
    SmartDashboard.putNumber("poity", poi.getY());
    
    t_TurretSubsystem.goToAngle(RobotBasedAngle);
    if(hoodUp.getAsBoolean()){h_HoodSubsystem.goToAngle(shootAngle);
      //s_ShooterSubsystem.setVelocity(vs * 2.2492 + 0.56157);
      double output = vs * 2.2492 + 0.56157;
      //double output = 0.00918838 * Math.pow(vs, 4) - 0.199587 * Math.pow(vs, 3) + 1.45776*Math.pow(vs, 2) - 2.20965* vs+ 5.3498;
      s_ShooterSubsystem.setVelocity(output);
    }
    else {h_HoodSubsystem.goToAngle(Constants.HoodConstants.forwardLimit);
      s_ShooterSubsystem.setVelocity(0);
    }
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

  public Pose2d RobotPoseAdjustedTolimelightTurret(Pose2d pose){
        double y = -Constants.Swerve.LIMELIGHT_TURRET_POSE_Y;
        double x =  Constants.Swerve.LIMELIGHT_TURRET_POSE_X;
        Rotation2d a = pose.getRotation();
        Pose2d returnpose = new Pose2d(pose.getX() + (a.getCos()* x) - (a.getSin() * y ), pose.getY() + (a.getCos()* y) - (a.getSin() * x ),  a);
        SmartDashboard.putNumber("2adjPosex", returnpose.getX());
           SmartDashboard.putNumber("2adjPosey", returnpose.getY());
           SmartDashboard.putNumber("2adjheading", a.getDegrees());
       return returnpose;

        // return new Pose2d(pose.getX(), pose.getY(), swervePoseEstimator.getEstimatedPosition().getRotation());
    }


  public double CalculateOffset(double Vrz, double Dx, double t){
    double Input = (Vrz* t)/Dx;

    if (Double.isNaN(Input)){
      Input = 0;//Constant based on data
    }

    return Math.atan(Input);
  }

  public double CalculateVy(double Dx){
    return 0.098438*Dx + 5.81997;
  }

  public double timeTillTarget(double Vy){
    double a = -g * 0.5;
    double b = Vy;
    double c = -(h-i);
    double Input = (b * b) - (4* a * c);
    if(Double.isNaN(Input)){
      Input = 0;//Constant based on data
    }

    return (-b) - Math.sqrt(Input);
  }


  public double CalculateVx(double distance, double Vy){
    double a = -g * distance * distance * 0.5;
    double b = Vy*distance;
    double c = -(h - i);
    double Input = (b * b) - 4*(a * c);
    if(Double.isNaN(Input)){
      Input = 0;//Constant based on data
    }

    return (2*(a)) / ((-b) - Math.sqrt(Input));
  }

  public double CalculateVs(double Vx, double Vy, double Vrx){
    double Input = ((Vx - Vrx) * (Vx - Vrx)) + (Vy * Vy);
    if (Double.isNaN(Input)){
      Input = 0;//Constant based on data
    }
    return Math.sqrt(Input);
  }

  public double CalculateShootAngle(double Vx, double Vy, double Vrx){
    double HoodAngle = Vy/(Vx - Vrx);

    if(Vy/(Vx - Vrx) == (Math.PI/2)){
      return 0;//Constant based on data
    }
    else if( Vy/(Vx - Vrx) == (-Math.PI/2) ){
      return 0;//Constant based on data
    }
    else{
      return Math.toDegrees(Math.atan(HoodAngle));
    }
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
