// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Vacuum;

public class GripPistonControl extends CommandBase {
  /** Creates a new GripPistonControl. */
  Arm m_arm; 

  public GripPistonControl(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setGripSolenoid(!m_arm.getGripSolenoid());
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
