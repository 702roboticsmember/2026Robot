package frc.robot;





import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    private final LIDARSubsystem l_lidarSubsystem = new LIDARSubsystem();
    private final XboxController driver = new XboxController(0);
    private final XboxController codriver = new XboxController(1);


    //driver buttons
    //private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
   // private final JoystickButton fastMode = new JoystickButton(driver, XboxController.Button.kB.value);
    //private final JoystickButton speedToggle = new JoystickButton(driver, XboxController.Button.kA.value);
    //private final JoystickButton intake = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);
    private final JoystickButton intakeOut = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton intakeIn = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton armOut = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton armIn = new JoystickButton(driver, XboxController.Button.kB.value);
    
    private final JoystickButton shoot = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    
   // private final JoystickButton pointPID = new JoystickButton(driver, XboxController.Button.kStart.value);

    //private final JoystickButton flywheel = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton climbUp = new JoystickButton(codriver, XboxController.Axis.kLeftTrigger.value);
    private final JoystickButton climbDown = new JoystickButton(codriver, XboxController.Axis.kRightTrigger.value);
    private final JoystickButton extendClimb = new JoystickButton(codriver, XboxController.Button.kY.value);
    private final JoystickButton releaseClimb = new JoystickButton(codriver, XboxController.Button.kX.value);
    private final JoystickButton grabClimb = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton retractClimb = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton AutoIntake = new JoystickButton(codriver, XboxController.Button.kStart.value);
    //private final  servoEngage = new POVButton(driver, Constants.Direction.UP.direction);

    
    
   // private final JoystickButton autoAim = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton autoAimHUB = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton autoAimPASS = new JoystickButton(driver, XboxController.Button.kBack.value);
    //private final JoystickButton autoAimCo = new JoystickButton(codriver, XboxController.Button.kY.value);
    
    private final JoystickButton Nest = new JoystickButton(codriver, 0);
    private final POVButton UP = new POVButton(driver, Constants.Direction.UP.direction);
    private final POVButton DOWN = new POVButton(driver, Constants.Direction.DOWN.direction);
    private final POVButton LEFT = new POVButton(driver, Constants.Direction.LEFT.direction);
    private final POVButton RIGHT = new POVButton(driver, Constants.Direction.RIGHT.direction);
    
    //codriver buttons
    // private final JoystickButton Intake = new JoystickButton(codriver, XboxController.Button.kA.value);

        //TODO still have the capability to move turret without auto aim
        //Intake left bumper, shoot right bumper, flywheel on andd off x, fast mode toggle on a, climber up left trigger, climber down right trigger, y be outtake, turn it on with the flywheel, x toggles on the passing shot, 
    
    public static double power = 1;
    public BooleanSupplier hoodUp = ()-> true;
    public static boolean robotCentric = false;
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Command> teamChooser;

    public static Locations currentPOI = Locations.BLUEHUB;
    private Field2d locField2d = new Field2d();
    public static DigitalInput ClimbSwitch = new DigitalInput(6);
    public static Trigger ClimbTrigger = new Trigger(()-> ClimbSwitch.get());
    //public static AnalogTrigger ClimbSwitch = new AnalogTrigger(0);
    

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

    // private Command ArmOut(){
    //     return Commands.runOnce(()->i_IntakeArmSubsystem.goToAngle(100), i_IntakeArmSubsystem);
    // }

    private Command ArmOut(){
        return Commands.run(()->i_IntakeArmSubsystem.goToAngle(100), i_IntakeArmSubsystem).withDeadline(new WaitCommand(0.3));
    }
    
    
    private Command IntakeIn() {
        return new ParallelCommandGroup(
            
            new InstantCommand(() -> i_IntakeSubsystem.setIntakeSpeed(0.5), i_IntakeSubsystem)
        );
    }
    private Command IntakeOut() {
        return new ParallelCommandGroup(
            new InstantCommand(() -> f_FloorIndexerSubsystem.setFloorIndexSpeed(-0.3), f_FloorIndexerSubsystem),
            new InstantCommand(() -> i_IntakeSubsystem.setIntakeSpeed(-0.5), i_IntakeArmSubsystem)
        );
    }
    private Command IntakeStop() {
        return new ParallelCommandGroup(
            new InstantCommand(() -> f_FloorIndexerSubsystem.setFloorIndexSpeed(0), f_FloorIndexerSubsystem),
            new InstantCommand(() -> i_IntakeSubsystem.setIntakeSpeed(0), i_IntakeSubsystem)
        );
    }

    private Command Shoot() {
        return new ParallelCommandGroup(
            new InstantCommand(()->hoodUp = ()-> true),
            // new RepeatCommand(new SequentialCommandGroup(
            //     //new InstantCommand(()->f_FloorIndexerSubsystem.move(-2), f_FloorIndexerSubsystem),
            //      new WaitCommand(0.3),
            //      new InstantCommand(()->f_FloorIndexerSubsystem.move(10), f_FloorIndexerSubsystem)
            // )),
            new InstantCommand(()->i_IndexerSubsystem.setVelocity(100), i_IndexerSubsystem),
            new InstantCommand(()->i_IntakeSubsystem.setIntakeSpeed(0.3), i_IntakeSubsystem),  
           new InstantCommand(()->f_FloorIndexerSubsystem.setVelocity(80), f_FloorIndexerSubsystem)
           
            );
        
    }
    private Command ShootOff() {
        return new ParallelCommandGroup(
            new SequentialCommandGroup(new InstantCommand(()->i_IndexerSubsystem.setVelocity(-10), i_IndexerSubsystem),
            new WaitCommand(0.5),
            new InstantCommand(()->i_IndexerSubsystem.setVelocity(0), i_IndexerSubsystem)),
            new InstantCommand(()->f_FloorIndexerSubsystem.move(-4), f_FloorIndexerSubsystem),
            new InstantCommand(()->i_IntakeSubsystem.setIntakeSpeed(0), i_IntakeSubsystem),   
           // new InstantCommand(()->s_ShooterSubsystem.setVelocity(0), s_ShooterSubsystem),
            new InstantCommand(()->hoodUp = ()-> false));
        
    }

    private Command AutoAim() {
        SmartDashboard.putBoolean("autorun", true);
        return new AutoAimCommand(t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem, hoodUp, false);
    }
    
    private Command AimAndShoot() {
        return new ParallelCommandGroup(
            new InstantCommand(() ->AutoAim()),
            new InstantCommand(() ->AimAtHub()), 
            new InstantCommand(() ->Shoot()),
            new WaitCommand(3),
            new InstantCommand(() ->ShootOff())
        );
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
        if(Constants.getAlliance())return new AutoAimCommand(Locations.BLUEHUB.location, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem, ()-> true);
        else return new AutoAimCommand(Locations.REDHUB.location, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem, ()->true);
    }

    private Command PASS(){
      
        return new AutoAimCommand(t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem, ()->true, true);
    }

    private Command ReleaseClimb(){
        return Commands.run(()->{
            c_ClimbSubsystem.goToPosOffset(-1);
            c_ClimbSubsystem.setServo(180);
        }, c_ClimbSubsystem).withDeadline(new WaitCommand(0.4));
    }

    private Command GrabClimb(){
        return new InstantCommand(()->{
            //c_ClimbSubsystem.goToPosOffset(1);
            c_ClimbSubsystem.setServo(90);
        }, c_ClimbSubsystem);
    }

    private Command ExtendClimb(){
        return Commands.run(()->c_ClimbSubsystem.goToPos(Constants.ClimbConstants.extendedAngle), c_ClimbSubsystem);
    }

    private Command RetractClimb(){
        return Commands.run(()->c_ClimbSubsystem.goToPos(Constants.ClimbConstants.retractedAngle), c_ClimbSubsystem);
    }

    private Command HoodDown(){
        return new InstantCommand(()-> h_HoodSubsystem.goToAngle(Constants.HoodConstants.forwardLimit));
    }

    private Command AutoShoot() {
        return new SequentialCommandGroup(Shoot(),
        new WaitUntilCommand(() -> !l_lidarSubsystem.indexer_full()),
        ShootOff());
    }

    public Command ClimbAuto() {
        return new SequentialCommandGroup(ExtendClimb());
    }

    public Command toPoint(Pose2d pose){
        if(Constants.getAlliance()){
            return new PointToPointPID(s_Swerve, pose);
        }else{
            return new PointToPointPID(s_Swerve, Constants.flipPose2d(pose));
        }
    }

    public Command ClimbDeadline(){
        return Commands.waitUntil(ClimbTrigger);
    }
    // private Command AutoAim() {
    //     return new ParallelCommandGroup(
    //         new TurretRotatePIDCommand(t_TurretSubsytem, )
    //     )
    // }

    public Command AutoIntake(){
        return new ParallelCommandGroup(new AutoIntakeCommand(
            ()->LimelightHelpersCameronEdition.getTX(Constants.limelightConstants.limelightFront),
            ()->LimelightHelpersCameronEdition.getTA(Constants.limelightConstants.limelightFront), 
            ()-> LimelightHelpersCameronEdition.getTV(Constants.limelightConstants.limelightFront), 
            s_Swerve, 
            0.3, 
            i_IntakeSubsystem));
    }


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve(t_TurretSubsystem);

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

        t_TurretSubsystem.setDefaultCommand(new TurretRotateManualCommand(() -> driver.getRightX(), t_TurretSubsystem));
        c_ClimbSubsystem.setDefaultCommand(new InstantCommand(()-> c_ClimbSubsystem.setSpeed(driver.getLeftTriggerAxis() - driver.getRightTriggerAxis()), c_ClimbSubsystem));

        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, 
        ()-> -driver.getRawAxis(1) * power, 
        ()-> -driver.getRawAxis(0) * power,
        ()-> -driver.getRawAxis(4) * power, 
        ()->false));

        // driver.setRumble(RumbleType.kBothRumble, 0.2);

       
        NamedCommands.registerCommand("Shoot", AutoShoot());
        NamedCommands.registerCommand("AimAtHub", AimAtHub());
        // NamedCommands.registerCommand("AimAndShoot", AimAndShoot());
        NamedCommands.registerCommand("IntakeNormal", IntakeIn());
        NamedCommands.registerCommand("StopIntake", IntakeStop());
        NamedCommands.registerCommand("IntakeOut", IntakeOut());
        NamedCommands.registerCommand("AutoIntake", AutoIntake());
        NamedCommands.registerCommand("Climb", ClimbAuto());
        NamedCommands.registerCommand("ClimbDeadline", ClimbDeadline());
        NamedCommands.registerCommand("ClimbGrab", GrabClimb());
        NamedCommands.registerCommand("ClimbRetract", RetractClimb());
        //ClimbGrab
        NamedCommands.registerCommand("ArmOut", ArmOut());
        NamedCommands.registerCommand("HoodDown", HoodDown());
        NamedCommands.registerCommand("toLT", toPoint(new Pose2d(Constants.Locations.BLUELT.location, new Rotation2d(Math.toRadians(180)))));//left trench
    
        //NamedCommands.registerCommand("P2Ptop", new PointToPointPID(s_Swerve, new Pose2d(null, null, null)));
        // NamedCommands.registerCommand("P2Pbottom", new PointToPointPID(s_Swerve, new Pose2d(null, null, null)));
        
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
        // speedToggle.toggleOnTrue(new InstantCommand(() -> RobotContainer.power = .25));
        // speedToggle.toggleOnFalse(new InstantCommand(() -> RobotContainer.power = 1));
        
        intakeOut.whileTrue(IntakeOut()); 
        intakeOut.onFalse(IntakeStop()); 
        intakeIn.whileTrue(IntakeIn());//new ParallelCommandGroup(new IntakeArmPID(0, i_IntakeArmSubsystem), new InstantCommand(() -> i_IntakeSubsystem.setIntakeSpeed(0))));  
        intakeIn.onFalse(IntakeStop());
        autoAimHUB.whileTrue(AimAtHub());
        autoAimHUB.onFalse(HoodDown());
        autoAimPASS.whileTrue(PASS());
        autoAimPASS.onFalse(HoodDown());
        
        // climbUp.whileTrue(new ClimbPIDCommand(0, c_ClimbSubsystem));
        // climbDown.whileTrue(new ClimbPIDCommand(Constants.ClimbConstants.extendedAngle, c_ClimbSubsystem));
       //shoot.onTrue(new InstantCommand(()-> f_FloorIndexerSubsystem.move(5), f_FloorIndexerSubsystem));
        shoot.whileTrue(Shoot().onlyWhile(Constants.Swerve.good));
        //shoot.whileTrue(new InstantCommand(()-> hoodUp= ()-> false));
        shoot.onFalse(ShootOff());
        AutoIntake.whileTrue(AutoIntake());

        //autoAim.whileTrue(AutoAim());
        //autoAimCo.whileTrue(AutoAim());
        UP.onTrue(wrapLocationChange(()-> nextAllianceLocation()));
        DOWN.onTrue(wrapLocationChange(()-> prevAllianceLocation()));
        RIGHT.onTrue(wrapLocationChange(()-> nextLocation()));
        LEFT.onTrue(wrapLocationChange(()-> prevLocation()));
        armOut.onTrue(ArmOut());

        grabClimb.onTrue(GrabClimb());
        releaseClimb.onTrue(ReleaseClimb());
        extendClimb.onTrue(ExtendClimb());
        retractClimb.onTrue(RetractClimb());
        ClimbTrigger.whileTrue(new InstantCommand(()->SmartDashboard.putBoolean("Climb Contact", true)));
        ClimbTrigger.whileFalse(new InstantCommand(()->SmartDashboard.putBoolean("Climb Contact", false)));
       
        
        
        // armIn.onTrue(Commands.runOnce(()->i_IntakeArmSubsystem.goToAngle(7), i_IntakeArmSubsystem));


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

        return new ParallelCommandGroup(
            new InstantCommand(()->Swerve.gyro.reset()),
            autoChooser.getSelected());
            
          }
    }
     




