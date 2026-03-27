package frc.robot;





import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Locations;
import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.FloorOffset;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TurretRotateManualCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.FloorIndexerSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeArmSubsytem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LIDARSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TurretSubsystem;


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
   
    //Subsystems
    private final IntakeSubsystem i_IntakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem s_ShooterSubsystem = new ShooterSubsystem();
    private final ClimbSubsystem c_ClimbSubsystem = new ClimbSubsystem();
    private final IndexerSubsystem i_IndexerSubsystem = new IndexerSubsystem();
    private final IntakeArmSubsytem i_IntakeArmSubsystem = new IntakeArmSubsytem();
    private final FloorIndexerSubsystem f_FloorIndexerSubsystem = new FloorIndexerSubsystem();
    private final TurretSubsystem t_TurretSubsystem = new TurretSubsystem();
    private final HoodSubsystem h_HoodSubsystem = new HoodSubsystem();
    private final LEDSubsystem l_LEDSubsystem = new LEDSubsystem();
    private final LIDARSubsystem l_lidarSubsystem = new LIDARSubsystem();
    private final XboxController driver = new XboxController(0);
    private final XboxController codriver = new XboxController(1);




    //Driver Buttons
    private final JoystickButton armin = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton armPartial = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton intakeOut = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton intakeIn = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton armOut = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton shoot = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton autoAimHUB = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton autoAimPASS = new JoystickButton(driver, XboxController.Button.kBack.value);
    
    private final POVButton UP = new POVButton(driver, Constants.Direction.UP.direction);
    private final POVButton DOWN = new POVButton(driver, Constants.Direction.DOWN.direction);
    private final POVButton LEFT = new POVButton(driver, Constants.Direction.LEFT.direction);
    private final POVButton RIGHT = new POVButton(driver, Constants.Direction.RIGHT.direction);
    
    
    
    
    //Co-driver Buttons
    private final JoystickButton extendClimb = new JoystickButton(codriver, XboxController.Button.kY.value);
    private final JoystickButton retractClimb = new JoystickButton(codriver, XboxController.Button.kX.value);
    private final JoystickButton Nest = new JoystickButton(codriver, 0);
    private final JoystickButton COSTART = new JoystickButton(codriver, XboxController.Button.kStart.value);
    private final JoystickButton COBACK = new JoystickButton(codriver, XboxController.Button.kBack.value);
    private final JoystickButton COLEFTBUMPER = new JoystickButton(codriver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton CORIGHTBUMPER = new JoystickButton(codriver, XboxController.Button.kRightBumper.value);
  
    private final POVButton COUP = new POVButton(codriver, Constants.Direction.UP.direction);
    private final POVButton CODOWN = new POVButton(codriver, Constants.Direction.DOWN.direction);
    private final POVButton COLEFT = new POVButton(codriver, Constants.Direction.LEFT.direction);
    private final POVButton CORIGHT = new POVButton(codriver, Constants.Direction.RIGHT.direction);
    

    
   
    //Variables
    public static boolean BLUE_ALLIANCE = false;
    public static double power = 1;
    public static double max = 1;
    public static double TurretGoal = 0;
    public static double CurrentAngle = 0;
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


    public static boolean getAlliance(){
        var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        if(alliance.get() == DriverStation.Alliance.Red){
                            BLUE_ALLIANCE = false;
                            return false;
                        }else{
                                BLUE_ALLIANCE = true;
                                return true;
                            }
                      
                    }
                    return false;
    }


    private Command ArmOut(){
        return Commands.run(()->i_IntakeArmSubsystem.goToAngle(100), i_IntakeArmSubsystem).withDeadline(new WaitCommand(0.3));
    }

    private Command ArmIn(){
        return new ParallelCommandGroup(Commands.run(()->i_IntakeArmSubsystem.goToAngle(5), i_IntakeArmSubsystem).withDeadline(new WaitCommand(0.3)), 
        HoodDown(), 
        Commands.run(()-> t_TurretSubsystem.goToAngle(0), t_TurretSubsystem)).withDeadline(new WaitCommand(1));
    }

    private Command ArmPartial(){
        return Commands.run(()->i_IntakeArmSubsystem.goToAngle(60), i_IntakeArmSubsystem).withDeadline(new WaitCommand(0.3));
    }
    
    
    private Command IntakeIn() {
        return new ParallelCommandGroup(
            
            new InstantCommand(() -> i_IntakeSubsystem.setIntakeSpeed(0.6), i_IntakeSubsystem)
        ).withDeadline(new WaitCommand(0.3));
    }
    private Command IntakeOut() {
        return new ParallelCommandGroup(
            new InstantCommand(() -> f_FloorIndexerSubsystem.setFloorIndexSpeed(-0.3), f_FloorIndexerSubsystem),
            new InstantCommand(() -> i_IntakeSubsystem.setIntakeSpeed(-0.5), i_IntakeArmSubsystem)
        );
    }
    private Command IntakeStop() {
        return new ParallelCommandGroup(
            new InstantCommand(() -> i_IntakeSubsystem.setIntakeSpeed(0), i_IntakeSubsystem),
            new InstantCommand(() -> f_FloorIndexerSubsystem.setFloorIndexSpeed(0), f_FloorIndexerSubsystem)
        );
    }

    private Command IntakeOff() {
        return new ParallelCommandGroup(
            new InstantCommand(() -> i_IntakeSubsystem.setIntakeSpeed(0), i_IntakeSubsystem)
        );
    }

    private Command Shoot() {
        double jamtime;
        double reset;
        jamtime = Timer.getFPGATimestamp();
        reset = 0;
        return new SequentialCommandGroup(
            Commands.run(()->{
                if (Math.abs(CurrentAngle - TurretGoal) < Constants.TurretConstants.allowedShootingTolerance) {
                        i_IndexerSubsystem.setVelocity(140);
                        f_FloorIndexerSubsystem.setVelocity(0);
                    }
                    else {
                        i_IndexerSubsystem.setVelocity(0);
                        f_FloorIndexerSubsystem.setVelocity(0);
                    }
             }, i_IndexerSubsystem, f_FloorIndexerSubsystem).withDeadline(new WaitCommand(0.1)),
             new WaitCommand(0.1),
             Commands.run(()->{
                 if (Math.abs(CurrentAngle - TurretGoal) < Constants.TurretConstants.allowedShootingTolerance) {
                        i_IndexerSubsystem.setVelocity(140);
                        f_FloorIndexerSubsystem.setVelocity(60);
                    }
                    else {
                        i_IndexerSubsystem.setVelocity(0);
                        f_FloorIndexerSubsystem.setVelocity(0);
                    }
             }, i_IndexerSubsystem, f_FloorIndexerSubsystem).withDeadline(new WaitCommand(0.1)),
             new WaitCommand(0.1),
            new ParallelCommandGroup(
            //Commands.run(()->i_IntakeSubsystem.setIntakeSpeed(0.5), i_IntakeSubsystem),
            Commands.run(()->{
                double jamTime = jamtime;
                double Reset = reset;
                if (l_lidarSubsystem.jam()) {
                    Reset = Timer.getFPGATimestamp();
                }
                
                if (Math.abs(CurrentAngle - TurretGoal) < Constants.TurretConstants.allowedShootingTolerance) {
                        i_IndexerSubsystem.setVelocity(140);
                    }
                    else {
                        i_IndexerSubsystem.setVelocity(0);
                    }
                if(i_IndexerSubsystem.getVelocity() < 5 || (f_FloorIndexerSubsystem.getVelocity() < 5 && f_FloorIndexerSubsystem.getVelocity() > 0) ){
                    
                }else{
                    jamTime = Timer.getFPGATimestamp();
                }
                    if(Timer.getFPGATimestamp() - jamTime < 1 && !l_lidarSubsystem.jam() && Timer.getFPGATimestamp() - reset > 0.5){
                    if (Math.abs(CurrentAngle - TurretGoal) < Constants.TurretConstants.allowedShootingTolerance) {
                        f_FloorIndexerSubsystem.setVelocity(60);
                    } else {
                        f_FloorIndexerSubsystem.setFloorIndexSpeed(-20);
                    }
                }else{
                    SmartDashboard.putBoolean("fixed", true);
                    f_FloorIndexerSubsystem.setVelocity(-40);
        
                }
                

            }, f_FloorIndexerSubsystem, i_IndexerSubsystem),
                new InstantCommand(() -> max = 0.13)
                ));
    }

    private Command ShootOff() {
        return new SequentialCommandGroup(new ParallelCommandGroup(
            
            new InstantCommand(()->i_IndexerSubsystem.setVelocity(-10), i_IndexerSubsystem),
            new FloorOffset(f_FloorIndexerSubsystem, -20),
            new InstantCommand(() -> power = 1),
            new InstantCommand(() -> max = 1)

           ).withDeadline(new WaitCommand(0.1)), new WaitCommand(0.1), new ParallelCommandGroup(
            
            new InstantCommand(()->i_IndexerSubsystem.setVelocity(-0), i_IndexerSubsystem),
            new InstantCommand(() -> power = 1),
            new InstantCommand(() -> max = 1)

           ));
        
    }

    private Command ShootOG() {
        return new SequentialCommandGroup(
            Commands.run(()->i_IndexerSubsystem.setVelocity(100), i_IndexerSubsystem).withDeadline(new WaitCommand(0.25)),
        new ParallelCommandGroup(
           Commands.run(()->i_IndexerSubsystem.setVelocity(100), i_IndexerSubsystem),
           Commands.run(()->i_IntakeSubsystem.setIntakeSpeed(0.3), i_IntakeSubsystem),
           Commands.run(()->f_FloorIndexerSubsystem.setVelocity(80), f_FloorIndexerSubsystem)
            
           
           ));
        
    }

    private Command ShootDeadline(){
        return Commands.waitUntil(()-> !l_lidarSubsystem.indexer_full());
    }

    private Command ShootDeadlineTime(){
        return new SequentialCommandGroup(new WaitCommand(1), ShootDeadline());
    }
    

    private Command AutoAim() {
        SmartDashboard.putBoolean("autorun", true);
        return new AutoAimCommand(t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem, false, ()-> getAlliance());
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
        
            return new AutoAimCommand(true, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem, ()-> getAlliance());
        
        
            //return new AutoAimCommand(true, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem);
    }

    private Command AimAtHubBlue(){
        
            return new AutoAimCommand(Constants.Locations.BLUEHUB.location, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem, ()-> getAlliance());
        
            //return new AutoAimCommand(true, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem);
    }

    private Command AimAtHubRed(){
            
            return new AutoAimCommand(Constants.Locations.REDHUB.location, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem, ()-> getAlliance());
        
            //return new AutoAimCommand(true, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem);
    }

    //  private Command AimAtHub(){
    //     if(Constants.getAlliance())return new AutoAimCommand(Locations.BLUEHUB.location, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem, ()-> true);
    //     else return new AutoAimCommand(Locations.REDHUB.location, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem, ()->true);
    // }

    private Command PASS(){
      
        return new AutoAimCommand(t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem, true, ()-> getAlliance());
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
        return Shoot();
    }

    public Command ClimbAuto() {
        return new SequentialCommandGroup(ExtendClimb());
    }



    public Command ClimbDeadline(){
        return Commands.waitUntil(ClimbTrigger);
    }


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
    private final Swerve s_Swerve = new Swerve();

    public RobotContainer() {
       
        
        
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
        l_LEDSubsystem.setDefaultCommand(
        new InstantCommand(()->
        {
            l_LEDSubsystem.DoTheRainbow(false);
            l_LEDSubsystem.LEDScroll(8,0,true);
            l_LEDSubsystem.LEDScroll(8, 8, false);
            

        }
        , l_LEDSubsystem));
        
        
        
        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, 
        ()-> MathUtil.clamp(-driver.getRawAxis(1) * power, -max, max) , 
        ()-> MathUtil.clamp(-driver.getRawAxis(0) * power, -max, max),
        ()-> -driver.getRawAxis(4) * power, 
        ()->false, ()-> getAlliance()));

        s_ShooterSubsystem.setDefaultCommand(new InstantCommand(()-> s_ShooterSubsystem.setSpeed(codriver.getLeftTriggerAxis()* 0.4) , s_ShooterSubsystem));

        // driver.setRumble(RumbleType.kBothRumble, 0.2);

       
        NamedCommands.registerCommand("Shoot", AutoShoot());
        NamedCommands.registerCommand("ShootOff", ShootOff());
        NamedCommands.registerCommand("AimAtHub", AimAtHub());
        NamedCommands.registerCommand("AimAtHubBlue", AimAtHubBlue());
        NamedCommands.registerCommand("AimAtHubRed", AimAtHubRed());

    // NamedCommands.registerCommand("AimAndShoot", AimAndShoot());
        NamedCommands.registerCommand("IntakeNormal", IntakeIn());
        NamedCommands.registerCommand("StopIntake", IntakeStop());
        NamedCommands.registerCommand("IntakeOut", IntakeOut());
        NamedCommands.registerCommand("AutoIntake", AutoIntake());
        NamedCommands.registerCommand("Climb", ClimbAuto());
        NamedCommands.registerCommand("ClimbDeadline", ClimbDeadline());
        NamedCommands.registerCommand("ClimbGrab", new InstantCommand());
        NamedCommands.registerCommand("ClimbRetract", RetractClimb());
        NamedCommands.registerCommand("ShootDeadline", ShootDeadlineTime());
        //ClimbGrab
        NamedCommands.registerCommand("ArmOut", ArmOut());
        NamedCommands.registerCommand("ArmPartial", ArmPartial());
        NamedCommands.registerCommand("HoodDown", HoodDown());

        t_TurretSubsystem.setDefaultCommand(t_TurretSubsystem.run(()-> codriver.getRawAxis(0)* 0.4));
      

        
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
       
        intakeOut.whileTrue(IntakeOut()); 
        intakeOut.onFalse(IntakeStop()); 
        intakeIn.whileTrue(IntakeIn());//new ParallelCommandGroup(new IntakeArmPID(0, i_IntakeArmSubsystem), new InstantCommand(() -> i_IntakeSubsystem.setIntakeSpeed(0))));  
        intakeIn.onFalse(IntakeStop());
        autoAimHUB.whileTrue(AimAtHub());
        autoAimHUB.onFalse(HoodDown());
        autoAimPASS.whileTrue(PASS());
        autoAimPASS.onFalse(HoodDown());
        armin.onTrue(ArmIn());
        armPartial.whileTrue(ArmPartial());
        armPartial.onFalse(ArmOut());
        
      
        shoot.whileTrue(Shoot());
  
        shoot.onFalse(ShootOff());

        UP.onTrue(wrapLocationChange(()-> nextAllianceLocation()));
        DOWN.onTrue(wrapLocationChange(()-> prevAllianceLocation()));
        RIGHT.onTrue(wrapLocationChange(()-> nextLocation()));
        LEFT.onTrue(wrapLocationChange(()-> prevLocation()));
        armOut.onTrue(ArmOut());

         COUP.whileTrue(Commands.run(()-> t_TurretSubsystem.goToAngle(0), t_TurretSubsystem));
         CODOWN.whileTrue(Commands.run(()-> t_TurretSubsystem.goToAngle(180), t_TurretSubsystem));
         COLEFT.whileTrue(Commands.run(()-> t_TurretSubsystem.goToAngle(90), t_TurretSubsystem));
         CORIGHT.whileTrue(Commands.run(()-> t_TurretSubsystem.goToAngle(-85), t_TurretSubsystem));
        // COSTART.whileTrue((new TeleopSwerve(s_Swerve, 
        // ()-> -driver.getRawAxis(1) * power, 
        // ()-> -driver.getRawAxis(0) * power,
        // ()-> -driver.getRawAxis(4) * power, 
        // ()-> true)));
        
        extendClimb.onTrue(ExtendClimb());
        retractClimb.onTrue(RetractClimb());
        // CORIGHTBUMPER.whileTrue(Shoot());
        // //shoot.whileTrue(new InstantCommand(()-> hoodUp= ()-> false));
        // CORIGHTBUMPER.onFalse(ShootOff());


        // COSTART.whileTrue(AimAtHub());
        // COSTART.onFalse(HoodDown());
        // COBACK.whileTrue(PASS());
        // COBACK.onFalse(HoodDown());
        

        // cograbClimb.onTrue(GrabClimb());
        // coreleaseClimb.onTrue(ReleaseClimb());
        // COLEFTBUMPER.whileTrue(new ShootDistCommand(3.3, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem));
        // CORIGHTBUMPER.whileTrue(new ShootDistCommand(8, t_TurretSubsystem, h_HoodSubsystem, s_ShooterSubsystem));

    }


    void nextLocation(){
        this.currentPOI = this.currentPOI.next();
    }

    void prevLocation(){
        this.currentPOI = this.currentPOI.prev();
    }

    void nextAllianceLocation(){
        this.currentPOI = this.currentPOI.AlianceNext(getAlliance());
    }

    void prevAllianceLocation(){
        this.currentPOI = this.currentPOI.AliancePrev(getAlliance());
    }


    
     public Command getAutonomousCommand() {

        return new ParallelCommandGroup(
            //new InstantCommand(()->Swerve.gyro.reset()),
            autoChooser.getSelected()).andThen(new ParallelCommandGroup(ShootOff(), ArmOut(), IntakeOff()).withDeadline(new WaitCommand(1)));
            
          }
    }
     




