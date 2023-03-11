// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.CANDevices;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.PneumaticChannels;





public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private TalonFX armMotorOne;
  private TalonFX armMotorTwo;
  private Solenoid extensionSolenoid;
  private Solenoid gripSolenoid;
  private Solenoid pincerSolenoid;
  private boolean eSolenoidState;
  private boolean gSolenoidState;
  private boolean pSolenoidState;

  //private Solenoid breakPiston;
  //private boolean pistonState;
  private CANCoder armEncoder;
  private DigitalInput highLimit = new DigitalInput(2);
  private DigitalInput lowLimit = new DigitalInput(3);

  //Arm Encoder Positions:
  //Lowest Position: 217
  //Upper Position: 17-18
  //Straight Down: 200

  public Arm() {

    // Import canSparkMax motors for arm, running in parallel
    armMotorOne = new TalonFX(CANDevices.armMotorOne);
    //armMotorTwo = new TalonFX(CANDevices.armMotorTwo);
    eSolenoidState = false;
    gSolenoidState = false;
    pSolenoidState = false;
    extensionSolenoid = new Solenoid(PneumaticChannels.PCMId, PneumaticsModuleType.REVPH, 2);
    gripSolenoid = new Solenoid(PneumaticChannels.PCMId, PneumaticsModuleType.REVPH, 3);
    pincerSolenoid = new Solenoid(PneumaticChannels.PCMId, PneumaticsModuleType.REVPH, 4);



    //breakPiston = new Solenoid(PneumaticChannels.PCMId, PneumaticsModuleType.REVPH, PneumaticChannels.deploySolenoidChannel);
    //armMotorTwo.follow(armMotorOne);
    //armMotorTwo.setInverted(true);
    armEncoder = new CANCoder(CANDevices.armEncoder);
    armMotorOne.setNeutralMode(NeutralMode.Brake);
    //armMotorTwo.setNeutralMode(NeutralMode.Brake);
  }

  //Arm motor
  public void runArm(double speed){
      armMotorOne.set(TalonFXControlMode.PercentOutput, speed);
  }

  //Arm Encoder
  public double getArmEncoder() {
    return armEncoder.getPosition();
  }

  public void setEncoderPosition(double pos) {
    armEncoder.setPosition(pos);
  }

  public void showArmEncoder() {
    SmartDashboard.putNumber("Arm Encoder", getArmEncoder());
  }

  public boolean getHighLimit() {
    return highLimit.get();
  }

  public boolean getLowLimit() {
    return lowLimit.get();
  }


  //Extension and Grip Solenoids for the arm
  public boolean getExtensionSolenoid() {
    return eSolenoidState;
  }

  public void setExtensionSolenoid(boolean state) {
    eSolenoidState = state;
    extensionSolenoid.set(state);
  }

  public boolean getGripSolenoid() {
    return gSolenoidState;
  }

  public void setGripSolenoid(boolean state) {
    gSolenoidState = state;
    gripSolenoid.set(state);
  }

  public boolean getPincerSolenoid(){
    return pSolenoidState;
  }

  public void setPincerSolenoid(boolean state){
    pSolenoidState = state;
    pincerSolenoid.set(state);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
