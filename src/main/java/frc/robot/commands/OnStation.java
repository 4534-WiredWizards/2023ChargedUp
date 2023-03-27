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

public class OnStation extends SequentialCommandGroup {
  /** Creates a new OnStation. */
  public OnStation(DriveSubsystem drive, Arm arm, Vacuum vacuum) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
 
      new FollowTrajectory(drive, AutoTrajectories.onStation, true)   //Moves 90 inches back to charge station
        // new ParallelCommandGroup(
      //new AutoArm(arm, 3).withTimeout(2) uncomment
        //   new FollowTrajectory(drive, AutoTrajectories.slightBack, true)
        // ),
      //new FollowTrajectory(drive, AutoTrajectories.onStationFront, true)

       
       

    );
  }

  
}
