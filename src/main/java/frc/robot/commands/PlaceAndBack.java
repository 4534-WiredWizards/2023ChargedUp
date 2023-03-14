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

public class PlaceAndBack extends SequentialCommandGroup {
  /** Creates a new AutoTesting. */
  public PlaceAndBack(DriveSubsystem drive, Arm arm, Vacuum vacuum) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
        new SetTongue(arm, true),
        new SuctionControl(vacuum).withTimeout(1),
        new AutoArm(arm, 4).withTimeout(3),
        new FollowTrajectory(drive, AutoTrajectories.toFront, true),
        new ExtensionPistonControl(arm).withTimeout(2),
        new SuctionControl(vacuum),
        new GripPistonControl(arm),
        new FollowTrajectory(drive, AutoTrajectories.back, true),
        new ParallelCommandGroup(
          new AutoArm(arm, 3).withTimeout(3),
          new FollowTrajectory(drive, AutoTrajectories.exitZone, true)
        )
       
       

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
