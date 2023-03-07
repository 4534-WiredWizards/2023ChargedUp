// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.AutoTrajectories;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vacuum;

public class AutoPath1 extends SequentialCommandGroup {
  /** Creates a new AutoPath1. */
  public AutoPath1(DriveSubsystem drive, Arm arm, Vacuum vacuum) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new ParallelCommandGroup(
        new RotateArm(arm, true).withTimeout(2),
        new FollowTrajectory(drive, AutoTrajectories.toFront, true)
      ),
      //new ReleasePiece(vacuum),
      new ParallelCommandGroup(
        new RotateArm(arm, false).withTimeout(2),
        new FollowTrajectory(drive, AutoTrajectories.exitZone, true)
      )
      //new FollowTrajectory(drive.AutoTrajectories.balance, true);
    );
  }

  
}
