// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  DriveSubsystem m_drive;
  private double oldpitch;
  private double currentpitch;
  private double adjust_speed = 0.1;

  public AutoBalance(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oldpitch = m_drive.getPitch();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentpitch = m_drive.getPitch();

    if (currentpitch > 3) {
      if (currentpitch < oldpitch) {
        m_drive.drive(adjust_speed, 0, 0, true);
      }
    }

    else if (currentpitch < -3) {
      if (currentpitch > oldpitch) {
        m_drive.drive(-adjust_speed, 0, 0, true);
      }
    }

    else {
      m_drive.drive(0, 0, 0, true);
    }
    oldpitch = currentpitch;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
