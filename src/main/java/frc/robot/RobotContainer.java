
package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Map;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.InputDevices;
import frc.robot.Constants.fancyJoystick;
import frc.robot.autonomous.AutoTrajectories;
import frc.robot.commands.drivetrain.CharacterizeDrive;
//import frc.robot.commands.drivetrain.QuickTurn
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.commands.drivetrain.OperatorControl;
import frc.robot.commands.drivetrain.QuickTurn;
//import frc.robot.commands.superstructure.Indexing.Waiting;
//import frc.robot.commands.superstructure.shooting.RampUpWithVision;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LedLights;
import frc.robot.subsystems.Vacuum;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClimbPiston;
import frc.robot.subsystems.DistanceEstimator;
import frc.robot.subsystems.Limelight;
//import frc.robot.subsystems.NewIntake;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;

import frc.robot.commands.RotateArm;
import frc.robot.commands.SqueezePincer;
//import frc.robot.commands.SecondSuction;
import frc.robot.commands.SuctionControl;
import frc.robot.commands.ChangeLEDS;
import frc.robot.commands.ZeroArm;
import frc.robot.commands.resetGyro;
import frc.robot.commands.AprilTagDrive;
import frc.robot.commands.ArmToPosition;
import frc.robot.commands.AutoTesting;
import frc.robot.commands.PlaceAndBackLeft;
import frc.robot.commands.PlaceAndBackRight;
import frc.robot.commands.PlaceAndStation;
import frc.robot.commands.ControlArmPiston;
//import frc.robot.commands.ControlNewIntake;
import frc.robot.commands.ControlShooterMotor;
import frc.robot.commands.ControlVacuum;
//import frc.robot.commands.ControlVacumm;
import frc.robot.commands.DisconnectGyro;
import frc.robot.commands.DoNothing;
import frc.robot.commands.ExtensionPistonControl;
import frc.robot.commands.GripPistonControl;
//import frc.robot.commands.PistonControl;

public class RobotContainer {

    /**
     * Establishes the controls and subsystems of the robot
     */

 
    public static XboxController m_joystick = new XboxController(1);
    public static XboxController m_fancyJoystick = new XboxController(0);

    SendableChooser<Command> chooser = new SendableChooser<>();

    //chooser.simpleDrive("Simple Drive", simpleDrive)

    public DriveSubsystem drive = new DriveSubsystem();
    //public Intake t_intake = new Intake();
    //public Shooter t_shooter = new Shooter();
    //public FeederWheel t_feeder = new FeederWheel();
    public Pneumatics t_pneumatics = new Pneumatics();
    //public ClimbMotor t_climbMotor = new ClimbMotor();
    public ClimbPiston t_ClimbPiston = new ClimbPiston();
    public Limelight t_limelight = new Limelight();
    public DistanceEstimator t_destimator = new DistanceEstimator();
    public Vacuum t_vacuum = new Vacuum();
    public Arm t_arm = new Arm();
    public LedLights l_ledLights = new LedLights();
    //public NewIntake t_newintake = new NewIntake();

    
    String trajectoryJSON = "paths/output/FrontRight.wpilib.json";
    Trajectory trajectory = new Trajectory();
    
    public RobotContainer() {
        //callibrates joysticks
        drive.setDefaultCommand(
            new OperatorControl(
                drive, 
                () -> m_fancyJoystick.getLeftY(), 
                () -> m_fancyJoystick.getLeftX(), 
                () -> m_fancyJoystick.getRawAxis(3), //4
                () -> m_fancyJoystick.getRawAxis(2),
                true
            )
        );

        /*
        shooter.setDefaultCommand(
            new RunCommand(() -> shooter.runShooterPercent(gamepad.getRawAxis(3) / 5), shooter)
        );
        */
        

        Shuffleboard.getTab("Shooter")
        .add("FeederSpeed", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();
        
        configureButtonBindings();

    }

    AnalogInput vaccum = new AnalogInput(1);

    public void updateShuffleBoard() {

        SwerveModuleState[] tempStates; 

        SmartDashboard.putNumber("Left  Y Joy", m_joystick.getLeftY());
        SmartDashboard.putNumber("Left  X Joy", m_joystick.getLeftX());
        //NOTE: Greg's Right X controller is on Axis 2
        SmartDashboard.putNumber("Right X Joy 1", m_fancyJoystick.getRawAxis(2));
        SmartDashboard.putNumber("Right X Joy 2", m_fancyJoystick.getRawAxis(3));
        SmartDashboard.putNumber("vaccum", vaccum.getValue());


        tempStates = drive.getModuleStates();
        SmartDashboard.putNumber("CANcoder S7 FL", tempStates[2].angle.getDegrees());
        SmartDashboard.putNumber("CANcoder S1 FR", tempStates[0].angle.getDegrees());
        SmartDashboard.putNumber("CANcoder S6 RL", tempStates[3].angle.getDegrees());
        SmartDashboard.putNumber("CANcoder S3 RR", tempStates[1].angle.getDegrees());
        SmartDashboard.putNumber("CANcoder D8 FL", tempStates[2].speedMetersPerSecond);
        SmartDashboard.putNumber("CANcoder D2 FR", tempStates[0].speedMetersPerSecond);
        SmartDashboard.putNumber("CANcoder D5 RL", tempStates[3].speedMetersPerSecond);
        SmartDashboard.putNumber("CANcoder D4 RR", tempStates[1].speedMetersPerSecond);
        

    }

    public void configureButtonBindings() {


        //2023 Commands

        //Vacume Control
        //new JoystickButton(m_joystick, InputDevices.btn_b).onTrue(new SuctionControl(t_vacuum));
        //new JoystickButton(m_joystick, InputDevices.btn_x).onTrue(new GripPistonControl(t_arm));   
        //new JoystickButton(m_joystick, InputDevices.btn_y).onTrue(new ExtensionPistonControl(t_arm));     
        //new JoystickButton(m_joystick, InputDevices.btn_a).whileTrue(new ControlShooterMotor(t_destimator));

        new JoystickButton(m_fancyJoystick, fancyJoystick.l1).onTrue(new resetGyro(drive));
        new JoystickButton(m_fancyJoystick, fancyJoystick.circle).onTrue(new QuickTurn(drive, Math.toRadians(180)));

        //Suctions
        //new JoystickButton(m_joystick, InputDevices.btn_b).onTrue(new SecondSuction(t_vacuum));

        //Pistons
        //new JoystickButton(m_joystick, InputDevices.btn_leftBumper).onTrue(new GripPistonControl(t_arm));   
        //new JoystickButton(m_joystick, InputDevices.btn_rightBumper).onTrue(new ExtensionPistonControl(t_arm));     

        // Arm Roatation
        //new Trigger(m_joystick).whileTrue(new RotateArm(t_arm, true));
        //new JoystickButton(m_joystick, InputDevices.btn_leftBumper).whileTrue(new RotateArm(t_arm, false));
        //new JoystickButton(m_joystick, InputDevices.btn_rightBumper).whileTrue(new RotateArm(t_arm, true));
        

        //Intake Motors 
        //new JoystickButton(m_joystick, InputDevices.btn_a).whileTrue(new ControlNewIntake(t_newintake, true));       
        //new JoystickButton(m_joystick, InputDevices.btn_y).whileTrue(new ControlNewIntake(t_newintake, false));
        new JoystickButton(m_joystick, InputDevices.btn_x).onTrue(new SuctionControl (t_vacuum)); //Both vacume cups
        new JoystickButton(m_joystick, InputDevices.btn_b).onTrue(new GripPistonControl(t_arm)); //Lower Jaw 
        new JoystickButton(m_joystick, InputDevices.btn_a).onTrue(new ExtensionPistonControl(t_arm)); //Actuate on the arm (extendy)
        new JoystickButton(m_joystick, InputDevices.btn_y).onTrue(new SqueezePincer(t_arm)); //Pincer on top of robot arm
        new JoystickButton(m_joystick, InputDevices.btn_leftBumper).onTrue(new ChangeLEDS(l_ledLights));



        new JoystickButton(m_joystick, InputDevices.btn_start).onTrue(new ZeroArm(t_arm));
        //new POVButton(m_joystick, 90).onTrue(new AprilTagDrive(t_destimator, drive));
        //new POVButton(m_joystick, 0).whileTrue(new RotateArm(t_arm, true));
        //new POVButton(m_joystick, 180).whileTrue(new RotateArm(t_arm, false));

        new POVButton(m_joystick, 0).onTrue(new ArmToPosition(t_arm, 1));
        new POVButton(m_joystick, 90).onTrue(new ArmToPosition(t_arm, 2));
        new POVButton(m_joystick, 180).onTrue(new ArmToPosition(t_arm, 3));
        new POVButton(m_joystick, 270).onTrue(new ArmToPosition(t_arm, 4));





    }

    public Command getAutonomousCommand() {

        SmartDashboard.putNumber("Initialized", 1);
        drive.resetPose(new Pose2d(0, 0, new Rotation2d(0)));
        //return new PlaceAndBackLeft(drive, t_arm, t_vacuum);
        //return new AutoTesting(drive, t_arm, t_vacuum);
        return new PlaceAndBackRight(drive, t_arm, t_vacuum);
        // return new CharacterizeDrive(drive);
        //return new LeftDriveBack(drive, t_shooter, t_intake, t_feeder, t_limelight); 
        //return new OneShotAuto(drive, t_shooter, t_intake, t_feeder, t_limelight); 
        //return new DriveBack(drive, t_shooter, t_limelight, t_feeder);

    }

    public Command getTrajectories() {

        // SmartDashboard.putNumber("Initialized", 1);
        // drive.resetPose(trajectory.getInitialPose());
        // return new AutoTesting(drive, t_arm, t_vacuum);

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
              trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
           } catch (IOException ex) {
              DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
              
        }
        drive.resetPose(trajectory.getInitialPose());
        return new FollowTrajectory(drive, trajectory, true);
    }


     
    }   

