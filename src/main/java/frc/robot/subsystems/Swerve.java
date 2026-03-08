package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpersCameronEdition;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Our main drive subsystem
 */
public class Swerve extends SubsystemBase {
    
    public SwerveModule[] swerveModules;
    public static AHRS gyro;
    public  RobotConfig config;
    public LimelightHelpersCameronEdition.PoseEstimate limelightMeasurement;
    public LimelightHelpersCameronEdition.PoseEstimate limelightMeasurementTurret;
     public LimelightHelpers.PoseEstimate limelightMeasurementTurret2;
    public TurretSubsystem t_Subsystem;
    public static SwerveDrivePoseEstimator swervePoseEstimator;
    


    public Swerve(TurretSubsystem t_Subsystem) {
        this.t_Subsystem = t_Subsystem;
        //limelightMeasurement =  LimelightHelpersCameronEdition.getBotPoseEstimate_wpiBlue(Constants.limelightConstants.limelightBack);
        gyro = new AHRS( NavXComType.kMXP_SPI);
        gyro.reset();
        try {
            config = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        swerveModules = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        
        if(gyro.isConnected() && !gyro.isCalibrating()){
            swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.KINEMATICS, getGyroYaw(), getModulePositions(), new Pose2d());
        }else{
            swervePoseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.KINEMATICS, new Rotation2d(Math.toRadians(0)), getModulePositions(), new Pose2d());
        }
        
        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            Constants.Swerve.PATHPLANNER_FOLLOWER_CONFIG,
            config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
      
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        if(alliance.get() == DriverStation.Alliance.Red){
                            }else{
                                
                            }
                      return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                  },
                  this // Reference to this subsystem to set requirements
          );
    }

    private void driveRobotRelative(ChassisSpeeds speeds) {
        var swerveModuleStates = Constants.Swerve.KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (int i = 0; i < swerveModuleStates.length; i++) {
            swerveModules[i].setDesiredState(swerveModuleStates[i], false);
        }
    }

    private ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    private void resetPose(Pose2d startingPosition) {
        SmartDashboard.putNumber("xi", startingPosition.getX());
        SmartDashboard.putNumber("yi", startingPosition.getY());
        SmartDashboard.putNumber("ai", startingPosition.getRotation().getDegrees());
        swervePoseEstimator.resetPosition(
                new Rotation2d(Math.toRadians(gyro.getAngle())),
                this.getModulePositions(),
                startingPosition);
                
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    public void driveAdjustedHeading(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, Rotation2d TurretOffset) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading().plus(TurretOffset))
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveModules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveModules) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swervePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
       
        return (Constants.Swerve.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - gyro.getYaw())
                : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public double getAcc() {
        return gyro.getAccelFullScaleRangeG();
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveModules) {
            mod.resetToAbsolute();
        }
    }

    public double getGyroVelX(){
        return gyro.getRobotCentricVelocityX();
    }
    public double getGyroVelY(){
        return gyro.getRobotCentricVelocityY();
    }
    public double getGyroVelZ(){
        return gyro.getRobotCentricVelocityZ();
    }

    public Pose2d limelightTurretPoseAdjustedToRobot(Pose2d pose){
        double y = Constants.Swerve.LIMELIGHT_TURRET_POSE_Y;
        double x =  -Constants.Swerve.LIMELIGHT_TURRET_POSE_X;
        Rotation2d a = pose.getRotation().minus(t_Subsystem.getAngle());
        
       Pose2d returnpose = new Pose2d(pose.getX() + (a.getCos()* x) - (a.getSin() * y ), pose.getY() + (a.getCos()* y) - (a.getSin() * x ), a);
       SmartDashboard.putNumber("ogPosex", pose.getX());
           SmartDashboard.putNumber("ogPosey", pose.getY());
           SmartDashboard.putNumber("ogheading", pose.getRotation().getDegrees());
       SmartDashboard.putNumber("adjPosex", returnpose.getX());
           SmartDashboard.putNumber("adjPosey", returnpose.getY());
           SmartDashboard.putNumber("adjheading", a.getDegrees());
           return returnpose;
        // return new Pose2d(pose.getX(), pose.getY(), swervePoseEstimator.getEstimatedPosition().getRotation());
    }

    public Pose2d RobotPoseAdjustedTolimelightTurret(Pose2d pose){
        double y = -Constants.Swerve.LIMELIGHT_TURRET_POSE_Y;
        double x =  Constants.Swerve.LIMELIGHT_TURRET_POSE_X;
        Rotation2d a = getHeading();
       return new Pose2d(pose.getX() + (a.getCos()* x) - (a.getSin() * y ), pose.getY() + (a.getCos()* y) - (a.getSin() * x ),  new Rotation2d(Math.toRadians(LimelightHelpersCameronEdition.getIMUData(Constants.limelightConstants.limelightTurret).robotYaw)));
        // return new Pose2d(pose.getX(), pose.getY(), swervePoseEstimator.getEstimatedPosition().getRotation());
    }
   

    // public void setHeadingToField(){
    //     Rotation2d rotate = LimelightHelpersCameronEdition.getBotPoseEstimate_wpiBlue(Constants.limelightConstants.limelightTurret).pose.getRotation();
    //     if (this.limelightMeasurement != null){
    //     setHeading(rotate);
    //     }
    // }
    // public void setposetoField(){
        
    //     if (this.limelightMeasurement != null){
    //     setPose(limelightMeasurement.pose);
    //     }
    // }

public void addmt1VisionMeasurement(LimelightHelpersCameronEdition.PoseEstimate mt1){
        boolean doRejectUpdate = false;
        if (mt1 != null){
         if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 2)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(mt1.avgTagDist > 2.3)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        swervePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(mt1.std[0] , mt1.std[1], mt1.std[2]));
        swervePoseEstimator.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
            SmartDashboard.putBoolean("ran", true);
            SmartDashboard.putNumberArray("std", mt1.std);
             SmartDashboard.putNumber("dist",  mt1.avgTagDist);
      }
        }
    }

    // public void addmt2VisionMeasurement(LimelightHelpersCameronEdition.PoseEstimate mt2){
    //     if (mt2 != null){
    //         if (mt2.tagCount >= 2) {
            
            
    //             Constants.Swerve.swervePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(mt2.std[0], mt2.std[1], mt2.std[2]));
    //             Constants.Swerve.swervePoseEstimator.addVisionMeasurement(
    //                 mt2.pose,
    //                 mt2.timestampSeconds
    //         );
            
    //         }
    //     }
    
    // }



    @Override
    public void periodic() {
        limelightMeasurement =  LimelightHelpersCameronEdition.getBotPoseEstimate_wpiBlue(Constants.limelightConstants.limelightBack);
        limelightMeasurementTurret =  LimelightHelpersCameronEdition.getBotPoseEstimate_wpiBlue(Constants.limelightConstants.limelightTurret);
        if(gyro.isConnected() && !gyro.isCalibrating())swervePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroYaw(), getModulePositions());
        
       



        SmartDashboard.putNumber("gyro", getHeading().getDegrees() );
        
        
        SmartDashboard.putNumber("Acc",this.getAcc());
        for (SwerveModule mod : swerveModules) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        if (this.limelightMeasurementTurret != null){
             Pose2d pose = limelightTurretPoseAdjustedToRobot(limelightMeasurementTurret.pose);
             limelightMeasurementTurret.pose = pose;
             addmt1VisionMeasurement(limelightMeasurementTurret); 
           }
           ChassisSpeeds speed = getRobotRelativeSpeeds();
           Constants.Swerve.speeds = speed;
        SmartDashboard.putNumber("chassisx", speed.vxMetersPerSecond);
        SmartDashboard.putNumber("chassisy", speed.vyMetersPerSecond);
        if (this.limelightMeasurement != null){   
            addmt1VisionMeasurement(limelightMeasurement); 
        }
    // Constants.Swerve.Robotpose = swervePoseEstimator.getEstimatedPosition();
    // Constants.TurretConstants.turretPose2d = RobotPoseAdjustedTolimelightTurret(swervePoseEstimator.getEstimatedPosition());
    
 
} 
    
   
}
    

