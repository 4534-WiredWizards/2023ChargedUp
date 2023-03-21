// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticChannels;
import frc.robot.subsystems.PressureSensor;

public class Pneumatics extends SubsystemBase {

  private PressureSensor t_presureSensor;
  
  private Compressor compressor;
  /** Creates a new Pneumatics. */
  public Pneumatics() {
    t_presureSensor= new PressureSensor();
    compressor = new Compressor(PneumaticChannels.PCMId, PneumaticsModuleType.CTREPCM);
    addChild("Compression", compressor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      


  }

  public void setCompressor(boolean set){
    if (set) {
      //compressor.enableAnalog(1000, 1001);
      compressor.enableDigital();
    }
    else {
      compressor.disable();
      
    }
  }

  public void updatePressureSensor() {
    //System.out.println("Updating pressure");
    // System.out.println("Pressure:" + compressor.getPressure());
    SmartDashboard.putNumber("Pressure", t_presureSensor.getPressure());
  }
}
