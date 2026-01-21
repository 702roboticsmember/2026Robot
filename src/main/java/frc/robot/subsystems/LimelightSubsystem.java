// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
  private String limelightName = "limelight";
  /** Creates a new LimelightSubsystem. */
  public double getTargetX() {
    return LimelightHelpers.getTX(limelightName);
  }

  public double getTargetY() {
    return LimelightHelpers.getTY(limelightName);
  }

  public double getTargetA() {
    return LimelightHelpers.getTA(limelightName);
  }

  public boolean IsTargetAvailable() {
    return LimelightHelpers.getTV(limelightName);
  }

  public void setPipeline(int value) {
    LimelightHelpers.setPipelineIndex(limelightName, value);
  }

  public int getPipeline() {
    return (int)Math.floor(LimelightHelpers.getCurrentPipelineIndex(limelightName));
  }

  public int getClassifier() {
    // return tclass.getNumber(0).intValue();
    return LimelightHelpers.getClassifierClassIndex(limelightName);
  }

  public double getBotPoseX() {
    // double pose[] = botpose.getDoubleArray(new double[6]);
    double pose[] = LimelightHelpers.getBotPose(limelightName);
    return pose[0];
  }

  public double getBotPoseY() {
    // double pose[] = botpose.getDoubleArray(new double[6]);
    double pose[] = LimelightHelpers.getBotPose(limelightName);
    return pose[1];
  }

  public double[] getBotPoseTeamRelative() {
    return LimelightHelpers.getBotPose_wpiBlue(limelightName);
    // var alliance = DriverStation.getAlliance();
    //   if (alliance.isPresent()) {
    //     if(alliance.get() == DriverStation.Alliance.Red){
          
    //       return botpose_wpired.getDoubleArray(new double[7]);
    //     }else{
    //       return botpose_wpiblue.getDoubleArray(new double[7]);
    //     }
    //   }
    //   return new double[7];
  }

  public Pose2d getBotPose2d(){
    double[] pose = getBotPoseTeamRelative();
    Pose2d botpose = pose.equals(new double[7]) || !IsTargetAvailable()? null: new Pose2d(pose[0], pose[1], new Rotation2d(pose[5]));
    return botpose;
  }

  /**
   * 3D transform of the robot in the coordinate system of the primary in-view AprilTag (array (6))
   * @return Returns a double array of 6 [tx, ty, tz, pitch, yaw, roll] (meters, degrees)
   */
  public double[] getBotPose_TargetSpace(){
    double[] pose = LimelightHelpers.getBotPose_TargetSpace(limelightName);
    return pose;
  }

  /**
   * ID of the primary in-view AprilTag
   * @return double but ID should be an integer if there is no value it will send -1.
   */
  public double getTid(){
    return LimelightHelpers.getFiducialID(limelightName);
  }


  // public double getBotPoseYTeamRelative() {
  //   double pose[] = RobotContainer.color == Color.kRed ? botpose_wpired.getDoubleArray(new double[6])
  //       : botpose_wpiblue.getDoubleArray(new double[6]);
  //   return pose[1];
  // }

  

  public double getTargetPos(int value){
    double pos[] = LimelightHelpers.getTargetPose_CameraSpace(limelightName);
    //double posR[] = targetpose_cameraspace.getDoubleArray(new double[6]);
    return pos[value];
  }

  public double getCameraPos(int value){
    double pos[] = LimelightHelpers.getCameraPose_TargetSpace(limelightName);
    return pos[value];
  }

  public double TargetDistance(){
    return Math.sqrt(Math.pow(getTargetPos(0), 2) + Math.pow(getTargetPos(1), 2));
  }

  // public void setCamMode(int value) {
  //   // camMode.setDouble(value);
  //   LimelightHelpers.set
  // }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LLX", getTargetPos(0));
    SmartDashboard.putNumber("LLZ", getTargetPos(2));
    SmartDashboard.putNumber("LLRY", getTargetPos(4));
    

    SmartDashboard.putNumber("tclass", getClassifier());
    SmartDashboard.putNumber("BotPoseX", getBotPoseX());
    SmartDashboard.getNumberArray("Limelightposeeeee", getBotPoseTeamRelative());
    SmartDashboard.getNumber("LimelightposeX", getBotPose2d()== null? 5: getBotPose2d().getX());
}
  public LimelightSubsystem() {}
}
