// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbPiston;
import frc.robot.Constants.fancyJoystick;


public class ControlArmPiston extends CommandBase {
  /** Creates a new ControlLeftArm. */
  private final ClimbPiston m_climbPiston;

  public ControlArmPiston(ClimbPiston climbPiston) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climbPiston = climbPiston;
    addRequirements(m_climbPiston);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (frc.robot.RobotContainer.m_fancyJoystick.getRawButton(fancyJoystick.se)) {
      m_climbPiston.setDeployPiston(!m_climbPiston.getDeployPiston());
    }
    // m_climbPiston.setRetractPiston(false);
    // } else if (frc.robot.RobotContainer.m_fancyJoystick.getRawButton(fancyJoystick.st)) {
    //   m_climbPiston.setRetractPiston(!m_climbPiston.getRetractPiston());
    //   m_climbPiston.setDeployPiston(false);
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
