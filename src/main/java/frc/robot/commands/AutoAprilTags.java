// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.AutoTrajectories;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DistanceEstimator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vacuum;


public class AutoAprilTags extends SequentialCommandGroup {
  /** Creates a new AutoAprilTags. */
  public AutoAprilTags(DistanceEstimator dEstimator, DriveSubsystem drive, Arm arm, Vacuum vacuum) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new ParallelCommandGroup(
        new ArmToPosition(arm, 2),
        new AprilTagDrive(dEstimator, drive)
      ),
      new AutoReleasePiece(vacuum),
      new FollowTrajectory(drive, AutoTrajectories.exitZoneLeft, true),
      new FollowTrajectory(drive, AutoTrajectories.onStationBack, true)
  );
  }

}

 
