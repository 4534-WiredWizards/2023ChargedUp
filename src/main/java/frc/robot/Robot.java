package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import org.photonvision.PhotonCamera;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autonomous.AutoTrajectories;
import frc.robot.commands.ControlCompressor;
import frc.robot.commands.ControlVacuum;
import frc.robot.commands.RotateArm;
import frc.robot.commands.SetTongue;
import frc.robot.subsystems.DistanceEstimator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
//import frc.robot.subsystems.Shooter;



public class Robot extends TimedRobot {
    
    private RobotContainer robotContainer;

    private Command autonomousCommand;
    private boolean inAuto;

    private ControlVacuum t_ControlVacuum;
    private RotateArm t_RotateArm;
    private SetTongue t_SetTongue;
    private ControlCompressor t_ControlCompressor;

    //private ChooseAuto autoChooser = new ChooseAuto();
  //String trajectoryJSON = "paths/Sam.wpilib.json";
  //Trajectory trajectory = new Trajectory();
  //private PhotonCamera pcamera = new PhotonCamera("photoncamera");
  private Command TwoBallSimple;
  private Command ThreeBallSimple;
  private Command DriveBack;
  private Command CenterDriveBack;
  private Command LeftDriveBack;
  private Command RightDriveBack;
   
  public SendableChooser<Command> autoChooser;
  
    @Override
    public void robotInit() {

        robotContainer = new RobotContainer();
        
        //Initializes Control Vacuum Command so that the loop of checking the solenoid states can run
        t_ControlVacuum = new ControlVacuum(robotContainer.t_vacuum);
        t_RotateArm = new RotateArm(robotContainer.t_arm, true);
        t_SetTongue = new SetTongue(robotContainer.t_arm, false);
        t_ControlCompressor = new ControlCompressor(robotContainer.t_pneumatics, robotContainer.t_psensor);
      
        autoChooser = new SendableChooser<Command>();
        NetworkTableInstance.getDefault();
        //Added to speed up auto running
        //new AutoTrajectories();
        //UsbCamera fisheye = CameraServer.startAutomaticCapture();
        // fisheye.setResolution(320, 240);
        //fisheye.setPixelFormat(PixelFormat.kMJPEG);
        robotContainer.t_pneumatics.setCompressor(true);

        robotContainer.t_vacuum.setVacuumState(false);
        autoChooser.setDefaultOption("ThreeBallSimple", ThreeBallSimple);
        autoChooser.addOption("TwoBallSimple", TwoBallSimple);
        autoChooser.addOption("DriveBack", DriveBack);
        autoChooser.addOption("CenterDriveBack", CenterDriveBack);
        autoChooser.addOption("LeftDriveBack", LeftDriveBack);
        autoChooser.addOption("RightDriveBack", RightDriveBack);
        SmartDashboard.putData("Auto Routines", autoChooser);
        robotContainer.getTrajectories();

    }  

    @Override
    public void robotPeriodic() {   

        CommandScheduler.getInstance().run();
        robotContainer.drive.updateSmartDashboard();

        robotContainer.t_psensor.updatePressureSensor();

        //Starts checking for updates to solonides
    
        t_ControlVacuum.execute();
        t_ControlCompressor.execute();
        //t_RotateArm.execute();

        // SrobotContainer.t_shooter.updateShooter();
        // robotContainer.t_feeder.updateSmartDashboard();
        robotContainer.t_limelight.updateLimelightInfo();
        robotContainer.t_destimator.updateShuffleboard();
        robotContainer.updateShuffleBoard();
        robotContainer.t_arm.showArmEncoder();
        robotContainer.t_vacuum.updateVacuumSensor();
        robotContainer.t_arm.periodic();



        //run the vacume command in the perodic loop
                

    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {

    }

    @Override
    public void autonomousInit() {
        //System.out.print("In auto init");
        autonomousCommand = robotContainer.getAutonomousCommand();
        // autonomousCommand = robotContainer.getAutonomousCommand(); //robotContainer.getTrajectories();   //autoChooser.getSelected();
        System.out.print("Got auto command");
        if (autonomousCommand != null) {
            //drive.resetPose(new Pose2d(0, 0, new Rotation2d(0)));
            autonomousCommand.schedule();
        }

    }

    @Override
    public void teleopInit() {

        if (autonomousCommand != null) {autonomousCommand.cancel();}
        CommandScheduler.getInstance().schedule(t_RotateArm);
        CommandScheduler.getInstance().schedule(t_SetTongue);
    }

    // @Override
    // public void testInit() {
    //     robotContainer.getTestCommand();
    // }

    @Override
    public void testPeriodic() {
        
    }

}
