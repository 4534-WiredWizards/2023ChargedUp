// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private double mountAngleDegrees =30;
  private double limelightHeight = 5;
  private double targetHeight = 25; 
  private double verticalAngleDegrees;
  private double distance;

  public Limelight() {
    //CameraServer.startAutomaticCapture();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (hasTarget()) {
      verticalAngleDegrees = getY();
      distance = (targetHeight - limelightHeight)/Math.tan(Math.toRadians(mountAngleDegrees) + Math.toRadians(verticalAngleDegrees));
    }
  }

  protected NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  protected NetworkTableEntry tv = limelightTable.getEntry("tv");
  protected NetworkTableEntry tx = limelightTable.getEntry("tx");
  protected NetworkTableEntry ty = limelightTable.getEntry("ty");
  protected NetworkTableEntry ta = limelightTable.getEntry("ta");
  protected NetworkTableEntry ts = limelightTable.getEntry("ts");
  protected NetworkTableEntry tl = limelightTable.getEntry("tl");
  protected NetworkTableEntry tshort = limelightTable.getEntry("tshort");
  protected NetworkTableEntry tlong = limelightTable.getEntry("tlong");
  protected NetworkTableEntry thor = limelightTable.getEntry("thor");
  protected NetworkTableEntry tvert = limelightTable.getEntry("tvert");

  public boolean hasTarget() {
    if (tv.getDouble(0) > 0) {
      return true;
    }
    else {
      return false;
    }
  }
  public double getX() {
    return tx.getDouble(0);
  }

  public double getY() {
    return ty.getDouble(0);
  }

  public double getArea() {
    return ta.getDouble(0);
  }

  public double getSkew() {
    return ts.getDouble(0);
  }

  public double getLatency() {
    return tl.getDouble(0);
  }

  public double getShortSide() {
    return tshort.getDouble(0);
  }

  public double getLongSide() {
    return tlong.getDouble(0);
  }

  public double getHorizontal() {
    return thor.getDouble(0);
  }

  public double getVertical() {
    return tvert.getDouble(0);
  }

  public double getDistance() {
    return distance;
  }

  public void setLEDMode(int mode) {
    limelightTable.getEntry("ledMode").setNumber(mode);
  }

  public void setPipeline(int pipeline) {
    limelightTable.getEntry("pipeline").setNumber(pipeline);
  }

  public void updateLimelightInfo() {
    SmartDashboard.putNumber("Limelight x", getX());
    SmartDashboard.putNumber("Limelight y", getY());
    SmartDashboard.putNumber("Limelight area", getArea());
    SmartDashboard.putNumber("Limelight skew", getSkew());
    SmartDashboard.putNumber("Limelight latency", getLatency());
    SmartDashboard.putNumber("Limelight Distance", getDistance());
    SmartDashboard.putBoolean("Limelight target", hasTarget());
  }
}
