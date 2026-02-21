package frc.robot;





import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.Locations;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final IntakeSubsystem i_IntakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem s_ShooterSubsystem = new ShooterSubsystem();
    private final ClimbSubsystem c_ClimbSubsystem = new ClimbSubsystem();
    private final IndexerSubsystem i_IndexerSubsystem = new IndexerSubsystem();
    private final IntakeArmSubsytem i_IntakeArmSubsystem = new IntakeArmSubsytem();
    private final FloorIndexerSubsystem f_FloorIndexerSubsystem = new FloorIndexerSubsystem();
    private final TurretSubsystem t_TurretSubsystem = new TurretSubsystem();
    private final HoodSubsystem h_HoodSubsystem = new HoodSubsystem();

    private final XboxController driver = new XboxController(0);
    private final XboxController codriver = new XboxController(1);


    //driver buttons
    //private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
   // private final JoystickButton fastMode = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton speedToggle = new JoystickButton(driver, XboxController.Button.kA.value);
    //private final JoystickButton intake = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);
    private final JoystickButton intakeOut = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton intakeIn = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton armOut = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton armIn = new JoystickButton(driver, XboxController.Button.kB.value);
    
    private final JoystickButton shoot = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    
   // private final JoystickButton pointPID = new JoystickButton(driver, XboxController.Button.kStart.value);

    //private final JoystickButton flywheel = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton climbUp = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);
    private final JoystickButton climbDown = new JoystickButton(driver, XboxController.Axis.kRightTrigger.value);
    //private final  servoEngage = new POVButton(driver, Constants.Direction.UP.direction);
    
    
    
    private final JoystickButton autoAim = new JoystickButton(driver, XboxController.Button.kY.value);
    
    private final JoystickButton Nest = new JoystickButton(codriver, 0);
    private final POVButton UP = new POVButton(driver, Constants.Direction.UP.direction);
    private final POVButton DOWN = new POVButton(driver, Constants.Direction.DOWN.direction);
    private final POVButton LEFT = new POVButton(driver, Constants.Direction.LEFT.direction);
    private final POVButton RIGHT = new POVButton(driver, Constants.Direction.RIGHT.direction);
    
    //codriver buttons
    // private final JoystickButton Intake = new JoystickButton(codriver, XboxController.Button.kA.value);

     //TODO post a boolean to smart dashboard if your able to shoot or not, red or green
        //TODO still have the capability to move turret without auto aim
        //Intake left bumper, shoot right bumper, flywheel on andd off x, fast mode toggle on a, climber up left trigger, climber down right trigger, y be outtake, turn it on with the flywheel, x toggles on the passing shot, 
    
    public static double power = 1;
    public BooleanSupplier hoodUp = ()-> true;
    public static boolean robotCentric = false;
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Command> teamChooser;

    public static Locations currentPOI = Locations.BLUEHUB;
    private Field2d locField2d = new Field2d();

    public void debugLocations() {
        SmartDashboard.putString("Currently Aiming at",
                String.format("%s ", currentPOI.label));
                locField2d.setRobotPose(new Pose2d(currentPOI.location, new Rotation2d(0)));
        SmartDashboard.putData("current location", locField2d);
    }
    

    // private Command IntakeOut() {
    //     return new SequentialCommandGroup(
        
    //      // new WaitCommand(1),
    //         i_IntakeSubsystem.spin(() -> Constants.IntakeConstants.intakeMotor)
    //         );
    // }
    
    private Command IntakeIn() {
        return new ParallelCommandGroup(
            
            new InstantCommand(() -> i_IntakeSubsystem.setIntakeSpeed(0.5))
        );
    }
    private Command IntakeOut() {
        return new ParallelCommandGroup(
            new InstantCommand(() -> f_FloorIndexerSubsystem.setFloorIndexSpeed(-0.3)),
            new InstantCommand(() -> i_IntakeSubsystem.setIntakeSpeed(-0.5))
        );
    }
    private Command IntakeStop() {
        return new ParallelCommandGroup(
            new InstantCommand(() -> f_FloorIndexerSubsystem.setFloorIndexSpeed(0)),
            new InstantCommand(() -> i_IntakeSubsystem.setIntakeSpeed(0))
        );
    }

    private Command Shoot() {
        return new ParallelCommandGroup(
        //     new SequentialCommandGroup(
        //     new InstantCommand(()->f_FloorIndexerSubsystem.move(10), f_FloorIndexerSubsystem).withDeadline(new WaitCommand(1)), 
        //     new InstantCommand(()->f_FloorIndexerSubsystem.move(-3), f_FloorIndexerSubsystem).withDeadline(new WaitCommand(1)),
         new InstantCommand(()->i_IndexerSubsystem.setSpeedPrimary(Constants.IndexerConstants.PrimarySpeed + 0.1), i_IndexerSubsystem),
        //     new WaitCommand(1)
        // ),
         //new InstantCommand(()->s_ShooterSubsystem.setVelocity(14), s_ShooterSubsystem),
        //     new InstantCommand(()->hoodUp = ()-> true)
             //new InstantCommand(()->i_IndexerSubsystem.setSpeedPrimary(), i_IndexerSubsystem),
           
            // Commands.repeatingSequence(new SequentialCommandGroup(new InstantCommand(()->f_FloorIndexerSubsystem.setFloorIndexSpeed(-0.1), f_FloorIndexerSubsystem),
            //  new InstantCommand(()->i_IndexerSubsystem.setSpeedPrimary(Constants.IndexerConstants.PrimarySpeed), i_IndexerSubsystem),
            // new WaitCommand(0.6),
            new InstantCommand(()->f_FloorIndexerSubsystem.setFloorIndexSpeed(Constants.IndexerConstants.FloorSpeed + 0.5), f_FloorIndexerSubsystem)
            //  new InstantCommand(()->i_IndexerSubsystem.setSpeedPrimary(-.10), i_IndexerSubsystem),
            // new WaitCommand(0.4))), 
            
            
            // new InstantCommand(()->s_ShooterSubsystem.setVelocity(14), s_ShooterSubsystem),
            // new InstantCommand(()->hoodUp = ()-> true)
            );
        
    }
    private Command ShootOff() {
        return new ParallelCommandGroup(
            new InstantCommand(()->i_IndexerSubsystem.setSpeedPrimary(0), i_IndexerSubsystem),
            new InstantCommand(()->f_FloorIndexerSubsystem.setFloorIndexSpeed(0), f_FloorIndexerSubsystem),
           // new InstantCommand(()->s_ShooterSubsystem.setVelocity(0), s_ShooterSubsystem),
            new InstantCommand(()->hoodUp = ()-> true));
        
    }

    private Command AutoAim() {
        SmartDashboard.putBoolean("autorun", true);
        return new AutoAimCommand( t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem, hoodUp);
    }
    
    private Command Nest() {
        return new InstantCommand(() -> h_HoodSubsystem.goToAngle(0));
    }
    private Command wrapLocationChange(Runnable r) {
        return Commands.runOnce(() -> {
            r.run();
            RobotContainer.this.debugLocations();
        });
    }

    private Command AimAtHub(){
        if(Constants.getAlliance())return new AutoAimCommand(Locations.BLUEHUB.location, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem, hoodUp);
        else return new AutoAimCommand(Locations.REDHUB.location, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem, hoodUp);
    }



    // private Command AutoAim() {
    //     return new ParallelCommandGroup(
    //         new TurretRotatePIDCommand(t_TurretSubsytem, )
    //     )
    // }


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    public RobotContainer() {
        Constants.Swerve.BLUE_ALLIANCE = Constants.getAlliance();
        
        Field2d field = new Field2d();
        SmartDashboard.putData("Field", field);
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            field.setRobotPose(pose);
        });
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            field.getObject("target pose").setPose(pose);
        });
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });

        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, 
        ()-> -driver.getRawAxis(1) * power, 
        ()-> -driver.getRawAxis(0) * power,
        ()-> -driver.getRawAxis(4) * power, 
        ()->robotCentric));

        driver.setRumble(RumbleType.kBothRumble, 0.2);

       
        NamedCommands.registerCommand("shoot", Shoot());
        NamedCommands.registerCommand("AimAtHub", AimAtHub());
        
        //s_ShooterSubsystem.setDefaultCommand(s_ShooterSubsystem.runCmd(()-> codriver.getRawAxis(2) * 1));
        t_TurretSubsystem.setDefaultCommand(t_TurretSubsystem.run(()-> codriver.getRawAxis(0)));
        //i_IndexerSubsystem.setDefaultCommand(i_IndexerSubsystem.spin(()-> codriver.getRawAxis(0) * 0.5));
        //f_FloorIndexerSubsystem.setDefaultCommand(f_FloorIndexerSubsystem.spin(()-> codriver.getRawAxis(0) * 0.5));

        
        configureButtonBindings();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        teamChooser = new SendableChooser<>();
        
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Team Chooser", teamChooser);
        SmartDashboard.putNumber("Input Distance", 0);
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        //zeroGyro.onTrue(new ParallelCommandGroup(new InstantCommand(() -> s_Swerve.zeroHeading()), new InstantCommand(()->s_Swerve.gyro.reset())));
        speedToggle.toggleOnTrue(new InstantCommand(() -> RobotContainer.power = .25));
        speedToggle.toggleOnFalse(new InstantCommand(() -> RobotContainer.power = 1));
        
        intakeOut.whileTrue(IntakeOut()); 
        intakeOut.onFalse(IntakeStop()); 
        intakeIn.whileTrue(IntakeIn());//new ParallelCommandGroup(new IntakeArmPID(0, i_IntakeArmSubsystem), new InstantCommand(() -> i_IntakeSubsystem.setIntakeSpeed(0))));  
        intakeIn.onFalse(IntakeStop());
        
        // climbUp.whileTrue(new ClimbPIDCommand(0, c_ClimbSubsystem));
        // climbDown.whileTrue(new ClimbPIDCommand(Constants.ClimbConstants.extendedAngle, c_ClimbSubsystem));
       //shoot.onTrue(new InstantCommand(()-> f_FloorIndexerSubsystem.move(5), f_FloorIndexerSubsystem));
        shoot.whileTrue(Shoot());
        //shoot.whileTrue(new InstantCommand(()-> hoodUp= ()-> false));
        shoot.onFalse(ShootOff());

        autoAim.whileTrue(AutoAim());
        UP.onTrue(wrapLocationChange(()-> nextAllianceLocation()));
        DOWN.onTrue(wrapLocationChange(()-> prevAllianceLocation()));
        RIGHT.onTrue(wrapLocationChange(()-> nextLocation()));
        LEFT.onTrue(wrapLocationChange(()-> prevLocation()));
        armOut.onTrue(Commands.runOnce(()->i_IntakeArmSubsystem.goToAngle(100), i_IntakeArmSubsystem));
        armIn.onTrue(Commands.runOnce(()->i_IntakeArmSubsystem.goToAngle(7), i_IntakeArmSubsystem));


         //pointPID.whileTrue(new PointToPointPID(s_Swerve, new Pose2d(new Translation2d(2,2),new Rotation2d(Math.toRadians(180)))));

    }


    void nextLocation(){
        this.currentPOI = this.currentPOI.next();
    }

    void prevLocation(){
        this.currentPOI = this.currentPOI.prev();
    }

    void nextAllianceLocation(){
        this.currentPOI = this.currentPOI.AlianceNext(Constants.getAlliance());
    }

    void prevAllianceLocation(){
        this.currentPOI = this.currentPOI.AliancePrev(Constants.getAlliance());
    }


    
     public Command getAutonomousCommand() {
        return new SequentialCommandGroup(new InstantCommand(() -> {
             s_Swerve.gyro.reset();

            // s_Swerve.zeroHeading();
        }));
     }
    }
     




