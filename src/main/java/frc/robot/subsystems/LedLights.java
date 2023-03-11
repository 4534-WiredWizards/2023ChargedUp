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
  private final Relay relayTwo = new Relay(1);
  private boolean relayOneState; 
  private boolean relayTwoState;

  //Led Spesfic States
  private boolean coneGiveState;


  //Arm Encoder Positions:
  //Lowest Position: 217
  //Upper Position: 17-18
  //Straight Down: 200

  public LedLights() {
    relayOneState = false;
    relayTwoState = false;
  }
    
    

   

    //Take in an array of four ones and zeros and return an array of 4 booleans
     //Example: [1,0,1,0] -> [true,false,true,false]
    public boolean[] convertToBoolean(int[] input){
        // Flip the array so that its inverted, because most significant bit is first currently
        // Example: [1,0,1,0] -> [0,1,0,1]
        int[] flippedArray = new int[4];
        for(int i = 0; i < 4; i++){
            if(input[i] == 1){
                flippedArray[i] = 0;
            }
            else{
                flippedArray[i] = 1;
            }
        }

        boolean[] output = new boolean[4];
        for(int i = 0; i < 4; i++){
            if(input[i] == 1){
                output[i] = true;
            }
            else{
                output[i] = false;
            }
        }
        return output;
    }    
    
    
   private void setRelayOneState(int[] input) {
    // Get the boolean array from the input array
    boolean[] output = convertToBoolean(input);

    // Bit One and Two, these can be independently on or both on or both off
    if(output[0] == true && output[1] == true){
        //If both bits are on set both forward and reverse to on
        relayOne.set(Relay.Value.kOn);
    }
    else if(output[0] == false && output[1] == false){
        //If both bits are off set both forward and reverse to off
        relayOne.set(Relay.Value.kOff);
    }
    else if(output[0] == true && output[1] == false){
            //If bit one is on and bit two is off set forward to on 
        relayOne.set(Relay.Value.kForward);
    } else if(output[0] == false && output[1] == true){
        //If bit one is off and bit two is on set reverse to on
        relayOne.set(Relay.Value.kReverse);
    }
   }
   
   public void setRelayTwoState(int[] input) {
    // Get the boolean array from the input array
    boolean[] output = convertToBoolean(input);

    // Bit One and Two, these can be independently on or both on or both off
    if(output[2] == true && output[3] == true){
        //If both bits are on set both forward and reverse to on
        relayTwo.set(Relay.Value.kOn);
    }
    else if(output[2] == false && output[3] == false){
        //If both bits are off set both forward and reverse to off
        relayTwo.set(Relay.Value.kOff);
    }
    else if(output[2] == true && output[3] == false){
            //If bit one is on and bit two is off set forward to on 
        relayTwo.set(Relay.Value.kForward);
    } else if(output[2] == false && output[3] == true){
        //If bit one is off and bit two is on set reverse to on
        relayTwo.set(Relay.Value.kReverse);
    }
   }

   public void setRelaysFromArray(int[] input){
       setRelayOneState(input);
       setRelayTwoState(input);
   }
    

    
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
