package frc.robot.autonomous;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class AutoTrajectories {

    /**
     * Defines the trajectories to be run through auto
     */

    // configures the maximum velocity and accel for the trajectories
    private final static TrajectoryConfig config = 
        new TrajectoryConfig(
            AutoConstants.maxVelMetersPerSec, 
            AutoConstants.maxAccelMetersPerSecondSq
        )
        .setKinematics(DriveConstants.kinematics);

    /**
     * Each trajectory is treated as a constant statically through the class
     * 
     * Trajectories are cubic clamped - generated with automatically defined slopes
     */
    public static Trajectory autoNavSlalomTrajectory = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(30), new Rotation2d(0)),
            List.of(
                new Translation2d(Units.inchesToMeters(59), Units.inchesToMeters(30)),
                new Translation2d(Units.inchesToMeters(83), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(90)),
                new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(90)),
                new Translation2d(Units.inchesToMeters(260), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(288), Units.inchesToMeters(20)),
                new Translation2d(Units.inchesToMeters(323), Units.inchesToMeters(29)),
                new Translation2d(Units.inchesToMeters(330), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(323), Units.inchesToMeters(87)),
                new Translation2d(Units.inchesToMeters(290), Units.inchesToMeters(95)),
                new Translation2d(Units.inchesToMeters(265), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(30)),
                new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(30)),
                new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(59), Units.inchesToMeters(100))
            ),
            new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(90), new Rotation2d(0)),
            config
        );

    public static Trajectory autoNavBounceTrajectory = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(45), Units.inchesToMeters(90), new Rotation2d(0)), 
            List.of(
                new Translation2d(Units.inchesToMeters(80), Units.inchesToMeters(90)),
                new Translation2d(Units.inchesToMeters(85), Units.inchesToMeters(150)),
                new Translation2d(Units.inchesToMeters(110), Units.inchesToMeters(75)),
                new Translation2d(Units.inchesToMeters(165), Units.inchesToMeters(75)),
                new Translation2d(Units.inchesToMeters(170), Units.inchesToMeters(215)),
                new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(130)),
                new Translation2d(Units.inchesToMeters(260), Units.inchesToMeters(130)),
                new Translation2d(Units.inchesToMeters(270), Units.inchesToMeters(275)),
                new Translation2d(Units.inchesToMeters(280), Units.inchesToMeters(240))
            ),
            new Pose2d(Units.inchesToMeters(320), Units.inchesToMeters(240), new Rotation2d(0)), 
            config
        );

    public static Trajectory autoNavBarrelTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(45), Units.inchesToMeters(90), new Rotation2d(0)),
            List.of(
                new Translation2d(Units.inchesToMeters(158), Units.inchesToMeters(78)),
                new Translation2d(Units.inchesToMeters(170), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(30)),
                new Translation2d(Units.inchesToMeters(115), Units.inchesToMeters(70)),
                new Translation2d(Units.inchesToMeters(160), Units.inchesToMeters(90)),
                new Translation2d(Units.inchesToMeters(262), Units.inchesToMeters(106)),
                new Translation2d(Units.inchesToMeters(275), Units.inchesToMeters(126)),
                new Translation2d(Units.inchesToMeters(248), Units.inchesToMeters(165)),
                new Translation2d(Units.inchesToMeters(195), Units.inchesToMeters(121)),
                new Translation2d(Units.inchesToMeters(280), Units.inchesToMeters(20)),
                new Translation2d(Units.inchesToMeters(330), Units.inchesToMeters(20)),
                new Translation2d(Units.inchesToMeters(330), Units.inchesToMeters(80))
            ),  
            new Pose2d(Units.inchesToMeters(45), Units.inchesToMeters(80), new Rotation2d(0)),
            config
        );

    public static Trajectory galacticSearchRedATrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(45), Units.inchesToMeters(96), new Rotation2d(0)),
            List.of(    
                new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(90)),
                new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(160), Units.inchesToMeters(140))
            ),
            new Pose2d(Units.inchesToMeters(370), Units.inchesToMeters(140), new Rotation2d(0)),
            config
        );

    public static Trajectory galacticSearchRedBTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(45), Units.inchesToMeters(96), new Rotation2d(0)),
            List.of(
                new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(120)),
                new Translation2d(Units.inchesToMeters(150), Units.inchesToMeters(30)),
                new Translation2d(Units.inchesToMeters(210), Units.inchesToMeters(120))
            ),
            new Pose2d(Units.inchesToMeters(370), Units.inchesToMeters(120), new Rotation2d(0)),
            config
    );

    public static Trajectory testTrajectory = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)),
            List.of(
                new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(30)),
                new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(15)),
                new Translation2d(Units.inchesToMeters(90), Units.inchesToMeters(0))
            ), 
            new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(30), new Rotation2d(0)),
            config
    );

    public static Trajectory practiceTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(32)),
            List.of(
                //new Translation2d(Units.inchesToMeters(60), new Rotation2d(0)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(-90)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(0))
                //new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(60))
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60)),
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(-10), Units.inchesToMeters(40), new Rotation2d(20)),
            config
    );

    public static Trajectory point_S = //starting position 0
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            List.of(
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(-90)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(0))
                //new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(60))
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60)),
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(-20), Units.inchesToMeters(100), new Rotation2d(Math.toRadians(-10))),
            config
    );

    public static Trajectory point_X = //starting position 1
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            List.of(
                //new Translation2d(Units.inchesToMeters(60), new Rotation2d(0)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(-90)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(0))
                //new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(60))
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60)),
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(-40), Units.inchesToMeters(-60), new Rotation2d(Math.toRadians(-10))),
            config
    );

    public static Trajectory point_3 = //starting position 2
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            List.of(
                //new Translation2d(Units.inchesToMeters(60), new Rotation2d(0)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(-90)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(0))
                //new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(60))
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60)),
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(-30), new Rotation2d(Math.toRadians(20))),
            config
    );

    public static Trajectory backUp = 
    TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            List.of(
                //new Translation2d(Units.inchesToMeters(-90), new Rotation2d(-2))
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(-90)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(0))
                new Translation2d(Units.inchesToMeters(-50), Units.inchesToMeters(10)),
                new Translation2d(Units.inchesToMeters(-90), Units.inchesToMeters(-10))
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60)),
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(-110), Units.inchesToMeters(20), new Rotation2d(Math.toRadians(0))),
            config
    );

    public static Trajectory leftBackUp = 
    TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            List.of(
                //new Translation2d(Units.inchesToMeters(5), Units.inchesToMeters(20))
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60)),
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(5), Units.inchesToMeters(130), new Rotation2d(Math.toRadians(0))),
            config
    );

    public static Trajectory rightBackUp = 
    TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            List.of(
                //new Translation2d(Units.inchesToMeters(60), new Rotation2d(0)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(-90)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(0))
                //new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(60))
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60)),
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(-10), Units.inchesToMeters(-110), new Rotation2d(Math.toRadians(-90))),
            config
    );

    public static Trajectory littleLeft =
    TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-115), Units.inchesToMeters(5), new Rotation2d(Math.toRadians(0))),
            List.of(
                //new Translation2d(Units.inchesToMeters(60), new Rotation2d(0)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(-90)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(0))
                //new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(60))
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60)),
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(-115), Units.inchesToMeters(15), new Rotation2d(Math.toRadians(0))),
            config
    );

    public static Trajectory position2 =
    TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            List.of(
                //new Translation2d(Units.inchesToMeters(60), new Rotation2d(0)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(-90)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(0))
                //new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(60))
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60)),
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(40), Units.inchesToMeters(60), new Rotation2d(Math.toRadians(0))),
            config
    );

    public static Trajectory toFront =
    TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            List.of(
                //new Translation2d(Units.inchesToMeters(60), new Rotation2d(0)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(-90)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(0))
                //new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(60))
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60)),
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(15), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            config
    );

    public static Trajectory back =
    TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            List.of(
                //new Translation2d(Units.inchesToMeters(60), new Rotation2d(0)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(-90)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(0))
                //new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(60))
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60)),
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(-15), Units.inchesToMeters(-2), new Rotation2d(Math.toRadians(0))),
            config
    );

    public static Trajectory moveRight =
    TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            List.of(
                //new Translation2d(Units.inchesToMeters(60), new Rotation2d(0)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(-90)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(0))
                //new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(60))
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60)),
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(88), new Rotation2d(Math.toRadians(0))),
            config
    );

    public static Trajectory leaveCommunity =
    TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            List.of(
                new Translation2d(Units.inchesToMeters(0), new Rotation2d(40)),
                new Translation2d(Units.inchesToMeters(80), new Rotation2d(0))
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(-90)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(0))
                //new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(60))
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60)),
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(80), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            config
    );

    

    public static Trajectory exitZone =
    TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            List.of(
                //new Translation2d(Units.inchesToMeters(60), new Rotation2d(0)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(-90)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(0))
                //new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(60))
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60)),
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(-120), Units.inchesToMeters(30), new Rotation2d(Math.toRadians(0))),
            config
    );

    public static Trajectory moveLeft =
    TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            List.of(
                //new Translation2d(Units.inchesToMeters(60), new Rotation2d(0)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(-90)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(0))
                //new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(60))
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60)),
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(90), new Rotation2d(Math.toRadians(0))),
            config
    );

    public static Trajectory onStation =
    TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            List.of(
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(44), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            config
    );

    public static Trajectory test1 =
    TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            List.of(
                new Translation2d(Units.inchesToMeters(30), new Rotation2d(0))
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(-90)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(0))
                //new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(60))
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60)),
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(30), new Rotation2d(Math.toRadians(90))),
            config
    );
    

    public static Trajectory test2 =
    TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            List.of(
                //new Translation2d(Units.inchesToMeters(60), new Rotation2d(0)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(-90)),
                //new Translation2d(Units.inchesToMeters(10), new Rotation2d(0))
                //new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(60))
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60)),
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(40), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(90))),
            config
    );


    public static Trajectory test3 =
    TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            List.of(
                new Translation2d(Units.inchesToMeters(-40), Units.inchesToMeters(40)),
                new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(-40))
                //new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(10))
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            config
    );



    public static Trajectory fullPath =
    TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            List.of(
                new Translation2d(Units.inchesToMeters(15), Units.inchesToMeters(0)),
                new Translation2d(Units.inchesToMeters(-15), Units.inchesToMeters(-2)),
                new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(60)),
                new Translation2d(Units.inchesToMeters(-90), Units.inchesToMeters(0)),
                new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(-60)),
                new Translation2d(Units.inchesToMeters(40), Units.inchesToMeters(0))

                //new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(10))
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(-50), Units.inchesToMeters(-2), new Rotation2d(Math.toRadians(0))),
            config
    );

    public static Trajectory part1 =
    TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Math.toRadians(0))),
            List.of(
                new Translation2d(Units.inchesToMeters(15), Units.inchesToMeters(0)),
                new Translation2d(Units.inchesToMeters(-15), Units.inchesToMeters(0)),
                new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(60))
                //new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(10))
                //new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0)),
            ),
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(60), new Rotation2d(Math.toRadians(0))),
            config
    );


   


   


    





    /*For competition - Being across from shooter (on right looking at it) and going backwards to collect
      Steps are to shoot 3x, and then turn around and go backwards, collecting the balls while moving
    public static Trajectory acrossFromShooterAndCollect = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(323.5 - 94.66)), 
            List.of (

            ), 
            end, 
            config
    );
    
    //For competition - Being across from shoot and going sideways to get out of way
    //Steps are to shoot 3x, and then turn around and go backwards at an angle 
    public static Trajectory acrossFromShooterAndCollect = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(323.5 - 94.66)), 
            List.of (

            ), 
            end, 
            config
    );

    //For competition - Starting at the middle, going to shoot, and then moving backwards to collect
    public static Trajectory acrossFromShooterAndCollect = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d( Units.inchesToMeters(120), Units.inchesToMeters(323.5/2.)), 
            List.of (

            ), 
            end, 
            config
    );

    //For competition - Starting at the middle, going to shoot, and then moving backwards at an angle to get out of way
    public static Trajectory acrossFromShooterAndCollect = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(323.5/2)), 
            List.of (

            ), 
            end, 
            config
    );

    //For competition - Starting at left (when looking at drivers), going to shoot, and then moving backwards to collect
    public static Trajectory acrossFromShooterAndCollect = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(94.66)), 
            List.of (

            ), 
            end, 
            config
    );

    //For competition - Starting at left (when looking at drivers), going to shoot, and then moving backwards at angle
        public static Trajectory acrossFromShooterAndCollect = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(94.66)), 
            List.of (

            ), 
            end, 
            config
    );

    */

}