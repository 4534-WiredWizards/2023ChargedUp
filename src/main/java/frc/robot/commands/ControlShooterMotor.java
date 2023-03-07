// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.InputDevices;
import frc.robot.subsystems.DistanceEstimator;
import frc.robot.subsystems.PoseEstimator;

public class ControlShooterMotor extends CommandBase {
  /** Creates a new ControlShooterMotor. */

  DistanceEstimator dEstimator ;
  double x_distance;
  private CANSparkMax testMotor;
  //PIDController pid = new PIDController(0.1, 0, 0);
  
  public ControlShooterMotor(DistanceEstimator dEstimator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dEstimator = dEstimator;
    testMotor = new CANSparkMax(15, MotorType.kBrushless);

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x_distance = dEstimator.getTranslation().getX();
    if (dEstimator.hasTarget()) {
      testMotor.set(0.04*x_distance);
      //testMotor.set(pid.calculate(x_distance));
    }
    else {
      testMotor.set(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    testMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!frc.robot.RobotContainer.m_joystick.getRawButton(InputDevices.btn_a)){
    return true;
  }
  else {
    return false;
  }
}

}
