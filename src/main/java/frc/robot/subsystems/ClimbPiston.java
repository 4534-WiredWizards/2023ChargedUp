// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.PneumaticChannels;


public class ClimbPiston extends SubsystemBase {
  /** Creates a new RightArm. */
  //private CANSparkMax rightMotor;
  private Solenoid deploySolenoid;
  //private Solenoid retractSolenoid;

  public ClimbPiston() {
    deploySolenoid = new Solenoid(PneumaticChannels.PCMId, PneumaticsModuleType.CTREPCM, PneumaticChannels.deploySolenoidChannel);
    //retractSolenoid = new Solenoid(PneumaticChannels.PCMId, PneumaticsModuleType.REVPH, PneumaticChannels.retractSolenoidChannel);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setDeployPiston(boolean state) {
    deploySolenoid.set(state);
  }

  // public void setRetractPiston(boolean state) {
  //   retractSolenoid.set(state);
  // }

  public boolean getDeployPiston() {
    return deploySolenoid.get();
  }

  // public boolean getRetractPiston()  {
  //   return retractSolenoid.get();
  // }
}
