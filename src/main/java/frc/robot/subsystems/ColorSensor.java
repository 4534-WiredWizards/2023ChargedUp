// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ColorSensor extends SubsystemBase {
  /** Creates a new ColorSensor. */

  private final I2C.Port i2CPort;
  private final ColorSensorV3 m_colorSensor;
  private ColorMatch m_colorMatcher;

  public ColorSensor() {
    i2CPort = I2C.Port.kOnboard;
    m_colorSensor = new ColorSensorV3(i2CPort);
    m_colorMatcher = new ColorMatch();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (detectedColor.red > detectedColor.blue) {
      SmartDashboard.putString("DetectedColor", "Red");
    }
    else if (detectedColor.blue > detectedColor.red) {
      SmartDashboard.putString("DetectedColor", "Blue");
    } 
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    
  }

  public Color senseColor() {
    return m_colorSensor.getColor();

  }
}
