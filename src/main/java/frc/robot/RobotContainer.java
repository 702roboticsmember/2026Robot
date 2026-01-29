package frc.robot;



import static edu.wpi.first.units.Units.Value;

import javax.xml.crypto.dsig.spec.HMACParameterSpec;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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

    private final XboxController driver = new XboxController(0);
    private final XboxController codriver = new XboxController(0);


    //driver buttons
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton fastMode = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton slowMode = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton intake = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);
    private final JoystickButton flywheel = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton climbUp = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton climbDown = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton autoAim = new JoystickButton(codriver, 0);
    private final JoystickButton shoot = new JoystickButton(codriver, XboxController.Button.kA.value);
    //codriver buttons
    // private final JoystickButton Intake = new JoystickButton(codriver, XboxController.Button.kA.value);
    public static double power = 1;
    public static boolean robotCentric = false;
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Command> teamChooser;
    

    //private Command setIntake(boolean value) {
    //    return new InstantCommand(
    //       () -> IntakeSubsystem.setArmSpeed(1)
    //     );
    // }

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

        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, 
        ()-> -driver.getRawAxis(1) * power, 
        ()-> -driver.getRawAxis(0) * power,
        ()-> -driver.getRawAxis(4) * power, 
        ()->robotCentric));

       
        //r_ReleaseSubsystem.setDefaultCommand(r_ReleaseSubsystem.run(()-> codriver.getRawAxis(3) * 0.3));
        
        //s_ShooterSubsystem.setDefaultCommand(s_ShooterSubsystem.runCmd(()-> codriver.getRawAxis(2) * 1));

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
        zeroGyro.onTrue(new ParallelCommandGroup(new InstantCommand(() -> s_Swerve.zeroHeading()), new InstantCommand(()->s_Swerve.gyro.reset())));
        slowMode.onTrue(new InstantCommand(() -> RobotContainer.power = .25));
        fastMode.onTrue(new InstantCommand(() -> RobotContainer.power = 1));         
        intake.whileTrue(new SequentialCommandGroup(new IntakeArmPID(120, i_IntakeSubsystem), new WaitCommand(1), new IntakeRollerCommand(i_IntakeSubsystem))); 
        intake.whileFalse(new ParallelCommandGroup(new IntakeArmPID(0, i_IntakeSubsystem)));        
        flywheel.toggleOnTrue(new InstantCommand(()-> s_ShooterSubsystem.setSpeed(1)));
        climbUp.whileTrue(new ClimbPIDCommand(0, c_ClimbSubsystem));
        climbDown.whileTrue(new ClimbPIDCommand(Constants.ClimbConstants.extendedAngle, c_ClimbSubsystem));
        shoot.onTrue(new InstantCommand(() -> {
            i_IndexerSubsystem.setSpeedPrimary(0.1);
            i_IndexerSubsystem.setSpeedSecondary(0.05);
        }));
        shoot.onFalse(new InstantCommand(() -> {
            i_IndexerSubsystem.setSpeedPrimary(0);
            i_IndexerSubsystem.setSpeedSecondary(0);
        }));
    }
    
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup((new InstantCommand(() -> {
             s_Swerve.gyro.reset();
            // s_Swerve.zeroHeading();
        })), autoChooser.getSelected());
    }
}
