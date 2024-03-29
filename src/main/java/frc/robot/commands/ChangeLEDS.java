// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LedLights;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ChangeLEDS extends CommandBase {
  /** Creates a new SqueezePincer. */
  LedLights l_ledLights;
  private boolean coneGiveState; 



  public ChangeLEDS(LedLights LedLights) {
    // Change leds from cone to cube
    l_ledLights = LedLights;
  }

  @Override
  public void initialize(){
    SetConeToggle(!coneGiveState);
  }



  //Togels
  public void SetConeToggle(boolean state){
    
    coneGiveState = state;
    if(coneGiveState){
      coneGive();
        // Print to console give code
        System.out.print("Setting LEDs to cone patern");
    }else{
        cubeGive();
        System.out.print("Setting LEDs to cube patern");
    }
  }


  public boolean getConeToggleState(){
      return coneGiveState;
  }

  //Colors
  public void coneGive(){
    int[] input = {1,0,0,0};
    l_ledLights.setPinsFromArray(input);
    System.out.print("Setting LEDs TO Cone");
    SmartDashboard.putString("WhatIWant", "Cone");
  }

  public void cubeGive(){
    int[] input = {0,1,1,0};
    l_ledLights.setPinsFromArray(input);
    System.out.print("Setting LEDs TO Cube");
    SmartDashboard.putString("WhatIWant", "Cube");

  } 

  public void auto(){
    int[] input = {0,0,1,0};
    l_ledLights.setPinsFromArray(input);
  }

  public void enabled(){
    int[] input = {0,0,0,1};
    l_ledLights.setPinsFromArray(input);
  }

  public void disabled(){
    int[] input = {0,0,0,0};
    l_ledLights.setPinsFromArray(input);
  }

 




  

 

  public void defense(){
    int[] input = {0,1,0,1};
    l_ledLights.setPinsFromArray(input);
  }

  public void charge(){
    int[] input = {0,1,1,0};
    l_ledLights.setPinsFromArray(input);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
