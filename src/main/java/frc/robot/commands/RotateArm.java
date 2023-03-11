// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.InputDevices;
import frc.robot.subsystems.Arm;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateArm extends CommandBase {
  /** Creates a new RotateArm. */
  Arm m_arm;
  private boolean isUp;
  private int positionNum;
  private double finalSpeed = 0.1;
  private double increment = finalSpeed/20;
  private double currentSpeed;
  private double setpointOne = 5;
  private double setpointTwo = 10;
  private double setpointThree = 15;
  PIDController armPID = new PIDController(0.5, 0, 0);
  ArmFeedforward feedforward = new ArmFeedforward(1, 1, 1, 0);



  public RotateArm(Arm arm, boolean isUp) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    this.isUp = isUp;
    addRequirements(m_arm);
  }

  public RotateArm(Arm arm, int target) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    positionNum = target;
    addRequirements(m_arm);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentSpeed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if (isUp) {
    //   m_arm.runArm(-armSpeed);
    // }
    // else {
    //   m_arm.runArm(armSpeed);
    // }

    // if (isUp && !m_arm.getHighLimit()) {
    //   m_arm.runArm(0);
    // } 
    // else
    //System.out.println("Running arm");
    if (frc.robot.RobotContainer.m_joystick.getRawAxis(2) > 0.75) {
    //if (isUp) {
      while (currentSpeed < finalSpeed) {
        m_arm.runArm(-currentSpeed);
        currentSpeed += increment;
      }
      System.out.println("Running arm");
      m_arm.runArm(-finalSpeed);
    }

    // if (!isUp && !m_arm.getLowLimit()) {
    //   m_arm.runArm(0);
    // } else
     else if (frc.robot.RobotContainer.m_joystick.getRawAxis(3) > 0.75) {
     //if (!isUp) {
      while (currentSpeed < finalSpeed) {
        m_arm.runArm(currentSpeed);
        currentSpeed += increment;
      }
      m_arm.runArm(finalSpeed);
    }

    else {
      m_arm.runArm(0);
    }

    //Using PID
    //m_arm.runArm(armPID.calculate(m_arm.getArmEncoder(), setpointOne));

    //Using PID and FeedForward
    //m_arm.runArm(armPID.calculate(m_arm.getArmEncoder(), setpointOne) + feedforward.calculate(setpointOne, 0));

    // if(frc.robot.RobotContainer.m_joystick.getRawButton(InputDevices.btn_leftBumper)){
    //   boolean isUp = true;
    //   m_arm.runArm(armSpeed, isUp);
    // }
    // if(frc.robot.RobotContainer.m_joystick.getRawButton(InputDevices.btn_rightBumper)){
    //   boolean isUp = false;
    //   m_arm.runArm(armSpeed, isUp);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.runArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if (frc.robot.RobotContainer.m_joystick.getRawAxis(2) < 0.75 && frc.robot.RobotContainer.m_joystick.getRawAxis(3) < 0.75){
    //if (frc.robot.RobotContainer.m_joystick.getRawButton(InputDevices.btn_leftBumper)  && frc.robot.RobotContainer.m_joystick.getRawButton(InputDevices.btn_rightBumper)){
      //return true;
    //}
    return false;
    //else {
    //  return false;
    //}
  }


}


  

