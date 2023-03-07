// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DistanceEstimator;
import frc.robot.subsystems.DriveSubsystem;

public class AprilTagDrive extends CommandBase {
  /** Creates a new AprilTagDrive. */
  DistanceEstimator m_dEstimator;
  DriveSubsystem m_drive;
  private double x_distance;
  private double y_distance;
  private double yaw;
  private double x_setpoint = 1;
  private double y_setpoint = -0.1;
  private double rot_setpoint = 0;
  private double xSpeed;
  private double ySpeed;
  private double rotSpeed;
  PIDController xController = new PIDController(0.8, 0, 0.1);
  PIDController yController = new PIDController(0.8, 0, 0.1);
  PIDController rotController = new PIDController(0.1, 0, 0);
  public AprilTagDrive(DistanceEstimator dEstimator, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_dEstimator = dEstimator;
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x_distance = m_dEstimator.getTranslation().getX();
    y_distance = m_dEstimator.getTranslation().getY();
    yaw = m_dEstimator.getYaw();

    xSpeed = xController.calculate(x_distance, x_setpoint);
    ySpeed = yController.calculate(y_distance, y_setpoint);
    rotSpeed = rotController.calculate(yaw, rot_setpoint);

    //m_drive.drive(-xSpeed, 0, 0, true);
    
    if (m_dEstimator.hasTarget()) {
      m_drive.drive(-xSpeed, -ySpeed, rotSpeed, true);
    }
    else {
      m_drive.drive(0, 0, 0, true);
    }
    
    //m_drive.drive(xSpeed, ySpeed, rotSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!m_dEstimator.hasTarget()) {
      return true;
    }
    else {
      return false;
    }
    
  }
}
