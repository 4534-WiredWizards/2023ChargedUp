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
      new SuctionControl(vacuum),
      new DoNothing().withTimeout(.4),
      new GripPistonControl(arm),
      new DoNothing().withTimeout(0.3),
      new ArmToPosition(arm, 5, true).withTimeout(3),
      new FollowTrajectory(drive, AutoTrajectories.toFront, true),
      new ExtensionPistonControl(arm),
      new DoNothing().withTimeout(0.2),
      new SuctionControl(vacuum),
      new DoNothing().withTimeout(1),
      new ParallelCommandGroup(
        new ExtensionPistonControl(arm), 
        new FollowTrajectory(drive, AutoTrajectories.onStation, true)
      ),
      // new FollowTrajectory(drive, AutoTrajectories.back, true),
      // new AutoArm(arm, 3).withTimeout(2),
      new ParallelCommandGroup(

        new AutoBalance(drive),
        new ArmToPosition(arm, 3).withTimeout(2)
      ),
      new DoNothing().withTimeout(.7),
      new AutoBalance(drive),
      new DoNothing().withTimeout(.7),
      new AutoBalance(drive),
      new DoNothing().withTimeout(.7),
      new AutoBalance(drive),
      new DoNothing().withTimeout(.7),
      new AutoBalance(drive),
      new DoNothing().withTimeout(1),
      new AutoBalance(drive),
      new DoNothing().withTimeout(1),
      new AutoBalance(drive)

        //Moves 50 inches back to charge station
        // new ParallelCommandGroup(
      //new AutoArm(arm, 3).withTimeout(2) uncomment
        //   new FollowTrajectory(drive, AutoTrajectories.slightBack, true)
        // ),
      //new FollowTrajectory(drive, AutoTrajectories.onStationFront, true)

       
       

    );
  }

}



