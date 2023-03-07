// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vacuum;

public class SuctionControl extends CommandBase {
  /** Creates a new SuctionControl. */
  Vacuum m_vacuum;
  public SuctionControl(Vacuum vacuum) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vacuum = vacuum;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_vacuum.setSolenoid1(!m_vacuum.getSolenoid1());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
