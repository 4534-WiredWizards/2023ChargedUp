// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.platform.can.AutocacheState;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ArmDownLeftBack;
import frc.robot.commands.ArmDownRightBack;
import frc.robot.commands.OnStation;
import frc.robot.commands.PlaceAndBackLeft;
import frc.robot.commands.PlaceAndBackRight;
import frc.robot.commands.PlaceAndStation;
import frc.robot.commands.DoNothingAuto;;

public class AutoChooser extends SubsystemBase {
  /** Creates a new AutoChooser. */

  public enum AutoMode {
    PlaceAndStation,
    PlaceAndBackLeft,
    PlaceAndBackRight,
    OnStation,
    ArmDownLeftBack,
    ArmDownRightBack,
    DoNothingAuto
  }

  private SendableChooser<AutoMode> autoChooser;
  private Command autoRoutine;
  private DriveSubsystem m_drive;
  private Arm m_arm;
  private Vacuum m_vacuum;

  public AutoChooser(DriveSubsystem drive, Arm arm, Vacuum vacuum) {
    m_drive = drive;
    m_arm = arm;
    m_vacuum = vacuum;
    autoChooser = new SendableChooser<AutoMode>();
    // autoChooser.setDefaultOption("Place and Station", AutoMode.PlaceAndStation);
    autoChooser.addOption("Place and Back Left", AutoMode.PlaceAndBackLeft);
    autoChooser.addOption("Place and Back Right", AutoMode.PlaceAndBackRight);
    autoChooser.addOption("DoNothingAuto", AutoMode.DoNothingAuto);
    // autoChooser.addOption("On Station", AutoMode.OnStation);
    // autoChooser.addOption("Arm Down Left Back", AutoMode.ArmDownLeftBack);
    // autoChooser.addOption("Arm Down Right Back", AutoMode.ArmDownRightBack);

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command getAuto() {

    AutoMode selectedAutoMode = (AutoMode) (autoChooser.getSelected());
    System.out.println("Running getAuto");
    switch (selectedAutoMode) {
      default:
      case PlaceAndStation:
        autoRoutine = new PlaceAndStation(m_drive, m_arm, m_vacuum);
        System.out.println("Place And Station Auto");
        break;

      case PlaceAndBackLeft:
        autoRoutine = new PlaceAndBackLeft(m_drive, m_arm, m_vacuum);
        System.out.println("Place And Left Auto");
        break;

      case PlaceAndBackRight:
        autoRoutine = new PlaceAndBackRight(m_drive, m_arm, m_vacuum);
        System.out.println("Place And Right Auto");
        break;

      case OnStation:
        autoRoutine = new OnStation(m_drive, m_arm, m_vacuum);
        System.out.println("On Station Auto");
        break;

      case ArmDownLeftBack:
        autoRoutine = new ArmDownLeftBack(m_drive, m_arm, m_vacuum);
        break;

      case ArmDownRightBack:
        autoRoutine = new ArmDownRightBack(m_drive, m_arm, m_vacuum);
        break;

      case DoNothingAuto:
        autoRoutine = new DoNothingAuto(m_drive);
        break;

    }
    return autoRoutine;
  }
}



