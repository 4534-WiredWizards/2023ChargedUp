// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vacuum;

public class ControlVacuum extends CommandBase {
  /** Creates a new ControlVacuum. */
  private double speed = .5;
  Vacuum m_vacuum;
  private double voltage_max = 4;
  public ControlVacuum(Vacuum vacuum) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vacuum =  vacuum;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_vacuum.getSolenoid1() || m_vacuum.getSolenoid2()) {

      // if (m_vacuum.getSensorVoltage() < voltage_max) {
        m_vacuum.setVacuumSpeed(speed);
      // }
      // else {
        // m_vacuum.setVacuumSpeed(0);

      // }
    }

    else {
      m_vacuum.setVacuumSpeed(0);
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
