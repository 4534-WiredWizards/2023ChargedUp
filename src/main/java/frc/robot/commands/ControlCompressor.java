// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.PressureSensor;

public class ControlCompressor extends CommandBase {
  /** Creates a new ControlCompressor. */
  Pneumatics m_pneumatics;
  PressureSensor m_sensor;
  private boolean firstCharge;
  public ControlCompressor(Pneumatics pneumatics) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pneumatics = new Pneumatics();
    m_sensor = new PressureSensor();
    firstCharge = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (firstCharge && m_sensor.getPressure() < 115) {
      m_pneumatics.setCompressor(true);
    }
    else {
      firstCharge = false;
      if (m_sensor.getPressure() < 60) {
        m_pneumatics.setCompressor(true);
      }
      else  {
        m_pneumatics.setCompressor(false);
      }
    }
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
