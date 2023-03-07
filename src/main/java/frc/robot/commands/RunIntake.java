// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;




// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.InputDevices;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.Intake;

// public class RunIntake extends CommandBase {
//   /** Creates a new RunIntake. */
//   private final Intake m_intake;
//   private final boolean m_runForward;
//   private double leftIntakeSpeed=1.0;
//   private double rightIntakeSpeed=1.0;
//   private double centerIntakeSpeed=1.0;

//   public RunIntake(Intake intake, boolean runForward) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     m_intake = intake;
//     m_runForward = runForward;
//     addRequirements(m_intake);

//     // adds left, right, and center intake speed as sliders to shuffleboard
   
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     System.out.println("Running intake");
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     // grabs speed from shuffleboard with 0.5 as default
  
//     // if runfoward is true, run intake based on left or right trigger
//     if (m_runForward) {
//       if (frc.robot.RobotContainer.m_joystick.getRawAxis(InputDevices.btn_leftTrigger) > 0.1) {
//         m_intake.setLeftMotor(leftIntakeSpeed, true); //-1
//       } else if (frc.robot.RobotContainer.m_joystick.getRawAxis(InputDevices.btn_rightTrigger) > 0.1) {
//         m_intake.setRightMotor(rightIntakeSpeed, true); //1 
//       } else {
//         m_intake.setCenterMotor(centerIntakeSpeed, true); //-1
//       }
//     } 
    
//     else {
//       // runfoward is false so run intake in reverse
//       if (frc.robot.RobotContainer.m_joystick.getRawAxis(InputDevices.btn_leftTrigger) > 0.1) {
//         m_intake.setLeftMotor(leftIntakeSpeed, false);
//       } else if (frc.robot.RobotContainer.m_joystick.getRawAxis(InputDevices.btn_rightTrigger) > 0.1) {
//         m_intake.setRightMotor(rightIntakeSpeed, false);
//       } else {
//         m_intake.setCenterMotor(centerIntakeSpeed, false);
//       }
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     // if finished or interrupted, set all intakes speed to 0
//     m_intake.setLeftMotor(0, true);
//     m_intake.setRightMotor(0, true);
//     m_intake.setCenterMotor(0, true);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     // return true if y and a are not held
//     if (!frc.robot.RobotContainer.m_joystick.getRawButton(InputDevices.btn_y) && !frc.robot.RobotContainer.m_joystick.getRawButton(InputDevices.btn_a)) {
//       return true;
//     }
    
//     return false;
//   }
// }
