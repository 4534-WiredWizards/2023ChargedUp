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

public class AutoTesting extends SequentialCommandGroup {
  /** Creates a new AutoTesting. */
  public AutoTesting(DriveSubsystem drive, Arm arm, Vacuum vacuum) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
        //new FollowTrajectory(drive, AutoTrajectories.test1, true),
        new FollowTrajectory(drive, AutoTrajectories.leaveCommunity, true)
        //new FollowTrajectory(drive, AutoTrajectories.back, true)
        //new FollowTrajectory(drive, AutoTrajectories.moveRight, true),
        //new FollowTrajectory(drive, AutoTrajectories.exitZone, true),
        //new FollowTrajectory(drive, AutoTrajectories.moveLeft, true),
        //new FollowTrajectory(drive, AutoTrajectories.onStation, true)

    );
  }

}

//Positive y is left
//Positive x is forward

//April Tags 2 and 7
//30 forward 
//30 backward
//90 right
//100 back
//90 left
//44 ward
