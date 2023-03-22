// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PressureSensor extends SubsystemBase {
  /** Creates a new Pr  essureSensor. */
  AnalogInput pressureSensor;
  public PressureSensor() {
    pressureSensor = new AnalogInput(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getAnalogPressure() {
    return pressureSensor.getValue();
    //This does not return the actual pressure. It returns a value that is proportional to the pressure.
  }

  public double getPressure() {
    return pressureSensor.getValue()*(200/Math.pow(2, 12))-5;
    //Returns pressure in psi
  }

  public void updatePressureSensor() {
    //System.out.println("Updating pressure");
    // System.out.println("Pressure:" + compressor.getPressure());
    SmartDashboard.putNumber("Pressure", getPressure());
    //needs to be in pressuresensor
  }


}
