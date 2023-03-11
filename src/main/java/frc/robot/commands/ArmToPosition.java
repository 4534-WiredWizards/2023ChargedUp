// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.InputDevices;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class ArmToPosition extends CommandBase {
  /** Creates a new ArmToPosition. */
  Arm m_arm;
  private boolean isUp;
  private int position;
  private double upperLimit = -150;
  private double lowerLimit = 0;
  private final double upperPos = -130;
  private final double middlePos = -128;
  private final double lowerPos = -5;
  private final double station = -115;
  private double setpoint;
  private double ramp = 0;
  private double increment = 0.2;

  private double offset = 1;
  //ArmFeedForward feedforward = new ArmFeedForward(1, 0, 0)
  PIDController armPID = new PIDController(0.005, 0.0015, 0);
  public ArmToPosition(Arm arm, boolean up) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    isUp = up;
    addRequirements(m_arm);
  }

  public ArmToPosition(Arm arm, int pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    position = pos;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (position == 1) {
      setpoint = upperPos; 
    }
    else if (position == 2) {
      setpoint = middlePos;
    }
    else if (position == 3) {
      setpoint = lowerPos;
    }
    ramp = 0;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (isUp && m_arm.getArmEncoder() > upperLimit) {
    //   m_arm.runArm(0);
    // } 
    // else if (isUp) {
    //   m_arm.runArm(armPID.calculate(m_arm.getArmEncoder(), setpointOne));
    // }

    // if (!isUp && m_arm.getArmEncoder() < lowerLimit) {
    //   m_arm.runArm(0);
    // } 
    // else if (!isUp) {
    //   m_arm.runArm(armPID.calculate(m_arm.getArmEncoder(), setpointOne));
    // }
    if (ramp < 1) {
      ramp += increment;
    }
    
    if (m_arm.getArmEncoder() < upperLimit || m_arm.getArmEncoder() > lowerLimit)  {
      m_arm.runArm(0);
    }
    else {
      m_arm.runArm(ramp * -0.6 * armPID.calculate(m_arm.getArmEncoder(), setpoint) );
      System.out.println("Running PID");
    }
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.runArm(0);
    System.err.println("Command has ended");
    if(!interrupted) {
      CommandScheduler.getInstance().schedule(new RotateArm(m_arm, true));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_arm.getArmEncoder() > (setpoint-offset) && m_arm.getArmEncoder() < (setpoint+offset)){
    return true;
    }
    else if (frc.robot.RobotContainer.m_joystick.getRawAxis(2) > 0.1 && frc.robot.RobotContainer.m_joystick.getRawAxis(3) > 0.1) {
       return true;
     }
    else {
      return false;
    }
}

}
