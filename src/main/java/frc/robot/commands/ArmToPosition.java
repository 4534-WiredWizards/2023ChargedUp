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
  private final double upperCone = -140;
  private final double middleCone = -105; 
  private final double lowerPos = -5;
  private final double station = -126; 
  private final double upperCube = -130;

  private boolean inAuto = false;
  private double setpoint;
  private double ramp;
  private double increment = 0.2;

  private double offset = 1;
  //ArmFeedForward feedforward = new ArmFeedForward(1, 0, 0)

  PIDController armPID;

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

  public ArmToPosition(Arm arm, int pos, boolean auto) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    position = pos;
    inAuto = auto;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (position == 1) {
      setpoint = upperCube;
    }
    else if (position == 2) {
      setpoint = station;
    }
    else if (position == 3) {
      setpoint = lowerPos;
    }
    else if (position == 4) {
      setpoint = middleCone;
    }
    else if (position == 5) {
      setpoint = upperCone; 
    }
    
    
    //ramp = 0;
    armPID = new PIDController(0.05, 0.01, 0.01);
    armPID.setTolerance(1);
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
    //if (ramp < 1) {
    //  ramp += increment;
    //}
    
    if (m_arm.getArmEncoder() < upperLimit)  {
      m_arm.runArm(-0.1);
    }
    else if(m_arm.getArmEncoder() > lowerLimit) {
      m_arm.runArm(0.1);
    }
    else {
      m_arm.runArm(-0.5 * armPID.calculate(m_arm.getArmEncoder(), setpoint) );
      System.out.println("Running Arm PID in Auto ");
    }
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.runArm(0);
    System.err.println("Command has ended");
    if(!interrupted && !inAuto) {
      CommandScheduler.getInstance().schedule(new RotateArm(m_arm, true));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (m_arm.getArmEncoder() > (setpoint-offset) && m_arm.getArmEncoder() < (setpoint+offset)){
    // return true;
    // }
    if (frc.robot.RobotContainer.m_joystick.getRawAxis(2) > 0.5 || frc.robot.RobotContainer.m_joystick.getRawAxis(3) > 0.5) {
       return true;
     }
    else {
      return armPID.atSetpoint();
    }
}

}

