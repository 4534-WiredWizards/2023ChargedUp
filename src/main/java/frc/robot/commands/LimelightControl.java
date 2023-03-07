// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class LimelightControl extends CommandBase {
  /** Creates a new LimelightControl. */
  Limelight m_limelight;
  private double x_distance;
  private CANSparkMax testMotor2;
  
  public LimelightControl(Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_limelight = limelight;
    testMotor2 = new CANSparkMax(17, MotorType.kBrushless);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_limelight.hasTarget()) {
      x_distance = m_limelight.getDistance();
      testMotor2.set(0.02*x_distance);
    }
    else {
      testMotor2.set(0);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    testMotor2.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
