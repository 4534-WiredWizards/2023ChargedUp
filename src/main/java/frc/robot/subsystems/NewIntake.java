// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.CANDevices;

// public class NewIntake extends SubsystemBase {
//   /** Creates a new NewIntake. */
//   private CANSparkMax motor1;
//   //private CANSparkMax motor2;
  
//   public NewIntake() {
//     motor1 = new CANSparkMax(CANDevices.intakeMotorOne, MotorType.kBrushless);
//     //motor2 = new CANSparkMax(CANDevices.intakeMotorTwo, MotorType.kBrushless);

//     //motor2.follow(motor1);
//     //motor2.setInverted(true);
//   }

//   public void setIntakeSpeed(double speed) {
//     System.out.println("Setting speed");
//     motor1.set(speed);
//     //motor2.set(-1*speed);
//   }

//   public double getSpeed() {
//     return motor1.get();
//   }



//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
