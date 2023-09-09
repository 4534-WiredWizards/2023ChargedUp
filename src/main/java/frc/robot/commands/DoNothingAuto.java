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

public class DoNothingAuto extends SequentialCommandGroup {
  /** Creates a new AutoTesting. */
  public DoNothingAuto(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
        new DoNothing().withTimeout(15)
        
       
       

    );
  }

}
