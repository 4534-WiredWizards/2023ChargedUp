// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.PneumaticChannels;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vacuum extends SubsystemBase {
  /** Creates a new Vacuum. */
  private CANSparkMax vMotor;
  private boolean vacuumState;
  PowerDistribution pd;
  private Solenoid solenoid1;
  private Solenoid solenoid2;

  private boolean solenoid1State;
  private boolean solenoid2State;

  
  private AnalogInput vacumeSensor = new AnalogInput(0);



  public Vacuum() {
    vMotor = new CANSparkMax(13, MotorType.kBrushless);
    vacuumState = false;
    pd = new PowerDistribution(2, ModuleType.kRev);



    // Solenoid deffintion
    solenoid1State = false;
    solenoid2State = false;
    solenoid1 = new Solenoid(PneumaticChannels.PCMId, PneumaticsModuleType.REVPH, 1);
    solenoid2 = new Solenoid(PneumaticChannels.PCMId, PneumaticsModuleType.REVPH, 0);

    //Solenoid 1 is for the vacuum motor
    //Solenoid 2 is the old piston, not currently being used



  }

  //Vacume speed items
  public void setVacuumSpeed(double speed) {
    vMotor.set(speed);
  }

  public double getSpeed(){
    return vMotor.get();
  }
  
  //Vacume sate items
  public boolean getVacuumState(){
    return vacuumState;
  }

  public void setVacuumState(boolean state) {
    vacuumState = state;
    pd.setSwitchableChannel(state);
  }


  //Solenoid items
  public boolean getSolenoid1() {
    return solenoid1State;
  }

  public void setSolenoid1(boolean state) {
    solenoid1State = state;
    solenoid1.set(state);
  }

  public boolean getSolenoid2() {
    return solenoid2State;
  }

  public void setSolenoid2(boolean state) {
    solenoid2State = state;
    solenoid2.set(state);
  }

  


  //Analog vacumeSensor 
  public double getSensorRaw() {
    return vacumeSensor.getValue();
  }

  public double getSensorVoltage() {
    return vacumeSensor.getVoltage();
  }

  public void updateVacuumSensor(){
    SmartDashboard.putNumber("Vacuum Sensor Raw: ", getSensorRaw());
    SmartDashboard.putNumber("Vacuum Sensor Voltage: ", getSensorVoltage());

  }



  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
