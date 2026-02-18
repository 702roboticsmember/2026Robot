package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
//import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

import static frc.lib.util.COTSTalonFXSwerveConstants.SDS.MK4i.*;

//public final class constants
//this is for the constants
//for your information
public final class Constants {
    public static final double CONTROLLER_DEADBAND = 0.05;//0.05

    public static final class LEDConstants {

        public static final int LEDPort = 0;
        public static final int BufferLength = 0;

    }

     public enum Direction {
        UP(0), RIGHT(90), DOWN(180), LEFT(270);

        int direction;

        private Direction(int direction) {
            this.direction = direction;
        }
     }

    public enum Locations {
        BLUEHUB(new Translation2d(4.62, 4.035), "Blue Alliance Hub"),
        BLUELEFT(new Translation2d(), "Blue Alliance left side"),
        BLUERIGHT(new Translation2d(),  "right Alliance right side"),
        REDHUB(new Translation2d(11.920, 4.035), "Red Alliance Hub"),
        REDLEFT(new Translation2d(), "Red Alliance left side"),
        REDRIGHT(new Translation2d(),  "Red Alliance right side");

        public final Translation2d location;
        public final String label;
        private Locations(Translation2d location, String label) {
            this.location = location;
            this.label = label;
        }

        public Locations next() {
            switch (this) {
                case BLUEHUB:
                    return BLUELEFT;
                case BLUELEFT:
                    return BLUERIGHT;
                case BLUERIGHT:
                    return REDHUB;
                case REDHUB:
                    return REDLEFT;
                case REDLEFT:
                    return REDRIGHT;
                case REDRIGHT:
                    return BLUEHUB;
            }

            throw new IllegalStateException("never reached");
        }

        public Locations prev() {
            switch (this) {
                case BLUEHUB:
                    return REDRIGHT;
                case BLUELEFT:
                    return BLUEHUB;
                case BLUERIGHT:
                    return BLUELEFT;
                case REDHUB:
                    return BLUERIGHT;
                case REDLEFT:
                    return REDHUB;
                case REDRIGHT:
                    return REDLEFT;
            }

            // should never be reached
            throw new IllegalStateException("never reached");
        }

        public Locations AliancePrev(Boolean BLUE_ALLIANCE) {
            if(BLUE_ALLIANCE){
            switch (this) {
                case BLUEHUB:
                    return BLUERIGHT;
                case BLUELEFT:
                    return BLUEHUB;
                case BLUERIGHT:
                    return BLUELEFT;
                case REDHUB:
                    return BLUERIGHT;
                case REDLEFT:
                    return BLUEHUB;
                case REDRIGHT:
                    return BLUELEFT;
                }
            }else{
                switch (this) {
                case BLUEHUB:
                    return REDRIGHT;
                case BLUELEFT:
                    return REDHUB;
                case BLUERIGHT:
                    return REDLEFT;
                case REDHUB:
                    return REDRIGHT;
                case REDLEFT:
                    return REDHUB;
                case REDRIGHT:
                    return REDLEFT;
                }
            }
            // should never be reached
            throw new IllegalStateException("never reached");
        }
    
   public Locations AlianceNext(Boolean BLUE_ALLIANCE) {
            if(BLUE_ALLIANCE){
            switch (this) {
                case BLUEHUB:
                    return BLUELEFT;
                case BLUELEFT:
                    return BLUERIGHT;
                case BLUERIGHT:
                    return BLUEHUB;
                case REDHUB:
                    return BLUERIGHT;
                case REDLEFT:
                    return BLUEHUB;
                case REDRIGHT:
                    return BLUELEFT;
                }
            }else{
                switch (this) {
                case BLUEHUB:
                    return REDLEFT;
                case BLUELEFT:
                    return REDRIGHT;
                case BLUERIGHT:
                    return REDHUB;
                case REDHUB:
                    return REDLEFT;
                case REDLEFT:
                    return REDRIGHT;
                case REDRIGHT:
                    return REDHUB;
                }
            }
            // should never be reached
            throw new IllegalStateException("never reached");
        }
    }

    public static boolean getAlliance(){
        var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        if(alliance.get() == DriverStation.Alliance.Red){
                            Constants.Swerve.BLUE_ALLIANCE = false;}else{
                                Constants.Swerve.BLUE_ALLIANCE = true;
                            }
                      return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
    }

    public static final class Swerve {
        public static SwerveDrivePoseEstimator swervePoseEstimator;
        public static boolean BLUE_ALLIANCE = getAlliance();
        public static final boolean INVERT_GYRO = true;
         public static final double DRIVE_GEAR_RATIO = 5.36; //L1: 7.13 - L2 5.9 - L3 5.36
        public static final double ANGLE_GEAR_RATIO = 18.75;
        public static final double LIMELIGHT_TURRET_POSE_Y = -0.238;
        public static final double LIMELIGHT_TURRET_POSE_X = -0.087;

        public static final COTSTalonFXSwerveConstants FALCON_500_CONSTANTS = Falcon500(DRIVE_GEAR_RATIO);
        /**
         * Units: Meters
         */
        public static final double TRACK_WIDTH = Units.inchesToMeters(27.75);

        /**
         * Units: Meters
         */
        public static final double BASE_WIDTH = Units.inchesToMeters(15.75);

        /**
         * Units: Meters
         */
        public static final double DRIVEBASE_DIAMETER = Math.sqrt(TRACK_WIDTH * TRACK_WIDTH + BASE_WIDTH * BASE_WIDTH);

        /**
         * Units: Meters
         */
        public static final double DRIVEBASE_RADIUS = DRIVEBASE_DIAMETER / 2f;

        public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * 0.048;

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(BASE_WIDTH / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(BASE_WIDTH / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-BASE_WIDTH / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-BASE_WIDTH / 2.0, -TRACK_WIDTH / 2.0));



        /* Module Gear Ratios */
       

        public static final InvertedValue ANGLE_MOTOR_INVERT = FALCON_500_CONSTANTS.angleMotorInvert;
        public static final InvertedValue DRIVE_MOTOR_INVERT = FALCON_500_CONSTANTS.driveMotorInvert;

        public static final SensorDirectionValue CANCODER_INVERT = FALCON_500_CONSTANTS.cancoderInvert;

        /**
         * Units: Volts
         */
        public static final int ANGLE_STATOR_CURRENT_LIMIT = 40;
        public static final int ANGLE_CURRENT_LIMIT = 25;
        public static final int ANGLE_CURRENT_THRESHOLD = 40;
        public static final double ANGLE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;
        public static final boolean ANGLE_ENABLE_STATOR_CURRENT_LIMIT = false;

        public static final int DRIVE_STATOR_CURRENT_LIMIT = 50;
        public static final int DRIVE_CURRENT_LIMIT = 35;//35
        public static final int DRIVE_CURRENT_THRESHOLD = 50;//60
        public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;
        public static final boolean DRIVE_ENABLE_STATOR_CURRENT_LIMIT = false;


/*
        public static final int STATOR_CURRENT_LIMIT = 50;
        public static final int CURRENT_LIMIT = 35;//35
        public static final int CURRENT_THRESHOLD = 50;//60
        public static final double CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final boolean ENABLE_STATOR_CURRENT_LIMIT = false;
*/

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0;

        public static final PIDConstants ANGLE_PID = new PIDConstants(FALCON_500_CONSTANTS.angleKP,
                FALCON_500_CONSTANTS.angleKI, FALCON_500_CONSTANTS.angleKD);
        public static final PIDConstants DRIVE_PID = new PIDConstants(0.5, 0.0, 0.0);

        /* Drive Motor Characterization Values From SYSID */
        public static final double DRIVE_KS = 0.32;//.32
        public static final double DRIVE_KV = 1.51;//1.51
        public static final double DRIVE_KA = 0.27;//.27

        /** Units: m/s */
        public static final double MAX_SPEED = 12;
        /** Units: radians/s */
        public static final double MAX_ANGULAR_VELOCITY = 8;

        /* Neutral Modes */
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3; 
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(230.889);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(28.916);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(58.436);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-0.527);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
        public static final PPHolonomicDriveController PATHPLANNER_FOLLOWER_CONFIG = new PPHolonomicDriveController(
                new PIDConstants(5, 0, 0), 
                new PIDConstants(3, 0, 0)
                // MAX_SPEED,
                // DRIVEBASE_RADIUS,
                // new ReplanningConfig()
                );

        public static final String ODOMETRY_LIMELIGHT_NAME = null;
       

    }
    
    //Intake Constants:
    public static final class IntakeConstants {
        public static final int intakeMotor = 13;
        public static final int armMotor = 14;
        public static final int STATOR_CURRENT_LIMIT = 50;
        public static final int CURRENT_LIMIT = 50;//35
        public static final int CURRENT_THRESHOLD = 35;//60
        public static final double CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ENABLE_CURRENT_LIMIT = false;
        public static final boolean ENABLE_STATOR_CURRENT_LIMIT = false;
        public static final int STATOR_CURRENT_LIMIT2 = 50;//50
        public static final int CURRENT_LIMIT2 = 35;//35
        public static final int CURRENT_THRESHOLD2 = 60;//60
        public static final double CURRENT_THRESHOLD_TIME2 = 0.1;
        public static final boolean ENABLE_CURRENT_LIMIT2 = true;
        public static final boolean ENABLE_STATOR_CURRENT_LIMIT2 = false;
		public static final double kP = 0;
        public static final double kI = 0;
		public static double kD;
        public static final boolean softLimitEnable = false;

        public static final double forwardLimit = 117;

        public static final double reverseLimit = 3;

        public static final double intakeArmConversion = 45 * 0.1;
    
    }
    
    
    //Turret Constants
    public static final class TurretConstants {
        public static final int TurretMotorID = 15;
        public static final PIDConstants anglePID = new PIDConstants(0, 0, 0);

        public static final int STATOR_CURRENT_LIMIT = 50;
        public static final int CURRENT_LIMIT = 35;//35
        public static final int CURRENT_THRESHOLD = 50;//60
        public static final double CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final boolean ENABLE_STATOR_CURRENT_LIMIT = false;
        public static final double kP = 0.057;
        public static final double kI = 0.0014;
        public static final double kD = 0.0042;
        public static final double Tolerance = 0.1;
                public static final boolean softLimitEnable = false;

        public static final double forwardLimit = 85;

        public static final double reverseLimit = 265;
        
        public static Pose2d turretPose2d = new Pose2d();

        public static final double TurretConversionRate = (50.0/14.0)*(130.0/20.0);
        public static final Angle initialAngle = null;
        public static final boolean LimitEnable = false;
        // public static final int forwardSoftLimit = 0;
        // public static final int reverseSoftLimit = 0;
        public static final int angleMotor = 0;

    }

    //Shooter Constants
    public static final class ShooterConstants {
        public static final int shooterMotor1 = 16;
        public static final int shooterMotor2 = 17;
        public static final int angleMotor = 18;
        public static final double conversion = 1.5;
     
        public static final int STATOR_CURRENT_LIMIT = 50;
        public static final int CURRENT_LIMIT = 35;//35
        public static final int CURRENT_THRESHOLD = 50;//60
        public static final double CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final boolean ENABLE_STATOR_CURRENT_LIMIT = false;

        public static final double CruiseVelocity = 0;
        public static final double Acceleration = 0;
        public static final double Jerk = 0;

        public static final double kS = .25;
        public static final double kV = .12;
        public static final double kA = .01;
        public static final double kP = .11;
        public static final double kI = 0;
        public static final double kD = 0;
    } 

    //Climb Constants
    public static final class ClimbConstants {
        public static final int climbMotor = 19;

        public static final double CLIMB_GEAR_RATIO = 45;
        public static final int STATOR_CURRENT_LIMIT = 50;
        public static final int CURRENT_LIMIT = 35;//35
        public static final int CURRENT_THRESHOLD = 50;//60
        public static final double CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final boolean ENABLE_STATOR_CURRENT_LIMIT = false;

        public static final double extendedAngle = 0;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final boolean softLimitEnable = false;

        public static final double forwardLimit = 0;

        public static final double reverseLimit = 0;

    }

    //Auto Constants
    public static final class AutoConstants { 

        public static final double kHeadingOffset = 90;
        // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 7.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5.2;
        public static final double kMaxAngularSpeedRadiansPerSecond = 3 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 4* Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
 
    }

    public static final class AutoAimConstants{

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

    }

     //Indexer Constants
    public static final class IndexerConstants {
        
        public static final double gearingMultiplier = 1;
        public static final int indexMotorFeeder = 20;
        public static final int indexMotorTurret1 = 21;
        public static final int indexMotorTurret2 = 22;
        public static final int STATOR_CURRENT_LIMIT = 50;
        public static final int CURRENT_LIMIT = 35;//35
        public static final int CURRENT_THRESHOLD = 50;//60
        public static final double CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final boolean ENABLE_STATOR_CURRENT_LIMIT = false;
        public static final double PrimarySpeed = 0.8;
        public static final double SecondarySpeed = PrimarySpeed;

        public static final int STATOR_CURRENT_LIMIT_SECONDARY = 50;
        public static final int CURRENT_LIMIT_SECONDARY = 35;
        public static final int CURRENT_THRESHOLD_SECONDARY = 50;
        public static final double CURRENT_THRESHOLD_TIME_SECONDARY = 0.1;
        public static final boolean ENABLE_CURRENT_LIMIT_SECONDARY = true;
        public static final boolean ENABLE_STATOR_CURRENT_LIMIT_SECONDARY = false;
        //Floor Indexer Constants
        ///public static final int FloorID = 0;
		public static final double FLOOR_STATOR_CURRENT_LIMIT = 0;
        public static final double FLOOR_CURRENT_LIMIT = 0;
        public static final boolean FLOOR_ENABLE_STATOR_CURRENT_LIMIT = false;
        public static final boolean FLOOR_ENABLE_CURRENT_LIMIT = false;
        public static final double FloorSpeed = 0.3;
    }

    public static final class HoodConstants {
        
        public static final int HoodMotor = 18;
        
        public static final int STATOR_CURRENT_LIMIT = 50;
        public static final int CURRENT_LIMIT = 35;//35
        public static final int CURRENT_THRESHOLD = 50;//60
        public static final double CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final boolean ENABLE_STATOR_CURRENT_LIMIT = false;
         public static final boolean softLimitEnable = false;

        public static final double forwardLimit = 63;

        public static final double reverseLimit = 46.2;
        public static final double initialAngle = 63.3;
        public static final double conversion = ((16.0/50.0) * (12.0/150.0));
    }
    
    public static final class PointToPointPIDConstants {

    public static final double tP = 2;
    public static final double tI = 0;
    public static final double tD = 0;

    public static final double aP = 0.05;
    public static final double aI = 0;
    public static final double aD = 0;

    public static final double tTolerance = 0.001;
    public static final double aTolerance = 0.1;













     }

     public static final class limelightConstants {

		public static final String limelightTurret = "limelight-duncan";        
        public static final String limelightFront = "limelight-douglas";
        public static final String limelightBack = "limelight-mega";
    }

    public static final class PhysicsConstants {
        public static final double gravity = 9.81;
        public static final double HubHeight = 1.89;
        public static final double BallIntialHeight = 0.4572;
        

    }
    }
    

