// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Relay;






public class LedLights extends SubsystemBase {
  /** Creates a new Arm. */

  private final Relay relayOne = new Relay(0);
  private boolean relayOneState; 
  //Arm Encoder Positions:
  //Lowest Position: 217
  //Upper Position: 17-18
  //Straight Down: 200

  public LedLights() {

    
    relayOneState = false;
    
    //armMotorTwo.setNeutralMode(NeutralMode.Brake);
  }

    public boolean getRelayOneState(){
        return relayOneState;
    }

    public void setRelayOneState(boolean state){
        relayOneState = state;
        if(relayOneState){
            relayOne.set(Relay.Value.kOn);
        }else{
            relayOne.set(Relay.Value.kOff);
        }
    }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
