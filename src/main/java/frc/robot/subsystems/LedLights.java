// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Relay;






public class LedLights extends SubsystemBase {
  /** Creates a new Arm. */


  private DigitalOutput DIO0 = new DigitalOutput(0);
  private DigitalOutput DIO1 = new DigitalOutput(1);
  private DigitalOutput DIO2 = new DigitalOutput(2);



  //Arm Encoder Positions:
  //Lowest Position: 217
  //Upper Position: 17-18
  //Straight Down: 200

 
    
    

   

    //Take in an array of four ones and zeros and return an array of 4 booleans
     //Example: [1,0,1,0] -> [true,false,true,false]
    public boolean[] convertToBoolean(int[] input){
        // Flip the array so that its inverted, because most significant bit is first currently
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
    
    
 

   public void setPinsFromArray(int[] input){
        try{
            boolean[] output = convertToBoolean(input);
            DIO0.set(output[0]);
            DIO1.set(output[1]);
            DIO2.set(output[2]);
        }
        catch (Exception e){
            System.out.println("Error in setPinsFromArray" + e.getMessage());
        }
   }
    

    
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
