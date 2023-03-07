// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TestDrive extends CommandBase {
  /** Creates a new TestDrive. */
  DriveSubsystem m_drive;
  PIDController xController = new PIDController(1, 0, 0);
  PIDController yController = new PIDController(1, 0, 0);
  PIDController rotController = new PIDController(0.5, 0, 0);
  private double x_speed;
  private double y_speed;
  private double rot_speed;

  public TestDrive(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x_speed = xController.calculate(8);
    y_speed = yController.calculate(8);
    rot_speed = rotController.calculate(8);

    m_drive.drive(x_speed, y_speed, rot_speed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
