// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.AutoTrajectories;
import frc.robot.commands.drivetrain.FollowTrajectory;
import frc.robot.commands.drivetrain.QuickTurn;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vacuum;

public class PlaceAndStation extends SequentialCommandGroup {
  /** Creates a new PlaceAndStation. */
  public PlaceAndStation(DriveSubsystem drive, Arm arm, Vacuum vacuum) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
        // new SetTongue(arm, true),
        new SuctionControl(vacuum),
        new DoNothing().withTimeout(0.2),
        new ExtensionPistonControl(arm),
        new DoNothing().withTimeout(0.2),
        new ExtensionPistonControl(arm),
        new DoNothing().withTimeout(0.2),
        new GripPistonControl(arm),
        new ArmToPosition(arm, 1, true).withTimeout(3),
        new FollowTrajectory(drive, AutoTrajectories.toFront, true),
        new ExtensionPistonControl(arm),
        new DoNothing().withTimeout(0.2),
        new SuctionControl(vacuum),
        new DoNothing().withTimeout(0.2),
        new ExtensionPistonControl(arm),
        // new ParallelCommandGroup(
        //   new AutoArm(arm, 3).withTimeout(3),
        //   new QuickTurn(drive, Math.PI)
        // ),
        new ParallelCommandGroup(
          new AutoArm(arm, 3).withTimeout(3),
          new FollowTrajectory(drive, AutoTrajectories.slightBack, true)
        ),
        new FollowTrajectory(drive, AutoTrajectories.onStationFront, true)

       
       

    );
  }

}



