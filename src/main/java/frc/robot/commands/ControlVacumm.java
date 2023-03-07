// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.InputDevices;
// import frc.robot.subsystems.Vacuum;

// public class ControlVacumm extends CommandBase{

//   Vacuum m_vacuum;
//   public ControlVacumm(Vacuum vacuum) {
//     // Use requires() here to declare subsystem dependencies
//     // eg. requires(chassis);
//     m_vacuum = vacuum;
//   }

//   // Called just before this Command runs the first time
//   @Override
//   public void initialize() {
//     System.out.println("Initializing");

//     // if(frc.robot.RobotContainer.m_joystick.getRawButton(InputDevices.btn_y)) {
//     // m_vacuum.setVacuumState(!m_vacuum.getVacuumState());
//     // }

//     // if(frc.robot.RobotContainer.m_joystick.getRawButton(InputDevices.btn_leftTrigger)) {
//     //   m_vacuum.setSuctionOne(!m_vacuum.getSuctionOne());
//     //   System.out.println("suction one changed");
//     // }

//     // if(frc.robot.RobotContainer.m_joystick.getRawButton(InputDevices.btn_rightTrigger)) {
//     //   m_vacuum.setSuctionTwo(!m_vacuum.getSuctionTwo());
//     //   System.out.println("suction two changed");

//     // }
//     if(frc.robot.RobotContainer.m_joystick.getRawButton(InputDevices.btn_leftTrigger)) {
//       m_vacuum.setConeSuction(!m_vacuum.getConeSuction());
//       System.out.println("Setting cone suction");
//     }

//     if(frc.robot.RobotContainer.m_joystick.getRawButton(InputDevices.btn_rightTrigger)) {
//       m_vacuum.setConeSuction(!m_vacuum.getCubeSuction());
//     }

//   }

//   // Called repeatedly when this Command is scheduled to run
//   @Override
//   public void execute() {
//     // if (m_vacuum.getState()) {
//     //   m_vacuum.setVacuumSpeed(0.5);
//     // }
//     // else {
//     //   m_vacuum.setVacuumSpeed(0);
//     // }
//     System.out.println("Executing");
//     m_vacuum.setVacuumState(true);
//     if (m_vacuum.getConeSuction()) {
//       m_vacuum.setCubeSuction(false);
//     }

//     if (m_vacuum.getCubeSuction()) {
//       m_vacuum.setConeSuction(false);
//     }

   
//   }

//   // Make this return true when this Command no longer needs to run execute()
//   @Override
//   public boolean isFinished() {
//     if (!m_vacuum.getConeSuction() && !m_vacuum.getCubeSuction()) {
//       return true;
//     }
//     else {
//       return false;
//     }
//   }

//   // Called once after isFinished returns true
//   @Override
//   public void end(boolean interupted) {
//     //m_vacuum.setState(false);
//     System.out.println("IS done");
//     m_vacuum.setVacuumState(false);
//   }
// }


