// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DistanceEstimator extends SubsystemBase {
  /** Creates a new DistanceEstimator. */
  PhotonCamera camera = new PhotonCamera("OV5647");
  double camera_height = Units.inchesToMeters(3.25);
  double target_height = Units.inchesToMeters(8);
  double camera_pitch = 0;
  double range;
  double trigDistance;
  double yaw;
  double pitch;
  double area;
  double skew;
  int targetID;
  Transform3d camToTargetTrans;
  private double previousTimeStamp = 0;
  boolean hasTarget = false;



  public DistanceEstimator() {
    camToTargetTrans = new Transform3d(new Translation3d(), new Rotation3d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    PhotonPipelineResult result = camera.getLatestResult();
    List<PhotonTrackedTarget> targets = result.getTargets();
    //System.out.println("Has targets: " + result.hasTargets());
    //System.out.println("Number of Targets: " + targets.size());
    double timeStamp = result.getTimestampSeconds();
    if (timeStamp != previousTimeStamp && result.hasTargets()) {
    previousTimeStamp = timeStamp;
    PhotonTrackedTarget target = result.getBestTarget();
    hasTarget = true;
    yaw = target.getYaw();
    pitch = target.getPitch();
    area = target.getArea();
    skew = target.getSkew();
    camToTargetTrans = target.getBestCameraToTarget();
    targetID = target.getFiducialId();
    range = PhotonUtils.calculateDistanceToTargetMeters(
            camera_height, 
            target_height,
            camera_pitch,
            Units.degreesToRadians(target.getPitch())
        );
    trigDistance = (target_height - camera_height)/Math.tan(camera_pitch + target.getPitch());
    System.out.println("Distance: " + range);
    }
    else if (!result.hasTargets()) {
      hasTarget = false;
    }
    }

    public double getID() {
      return targetID;
    }

    public double getDistance() {
      return range;
    }

    public double getYaw() {
      return yaw;
    }

    public double getArea() {
      return area;
    }

    public double getPitch() {
      return pitch;
    }


    public Rotation3d getRotation() {
      return camToTargetTrans.getRotation();
    }

    public Translation3d getTranslation() {
      return camToTargetTrans.getTranslation();
    }

    public boolean hasTarget() {
      return hasTarget;
    }


    public void updateShuffleboard() {
      SmartDashboard.putNumber("AprilTagDistance", getDistance());
      SmartDashboard.putNumber("AprilTagYaw", getYaw());
      SmartDashboard.putNumber("AprilTagArea", getArea());
      SmartDashboard.putNumber("AprilTagPitch", getPitch());
      SmartDashboard.putNumber("TargetID", getID());
      SmartDashboard.putNumber("AprilTagX", getTranslation().getX());
      SmartDashboard.putNumber("AprilTagY", getTranslation().getY());
      SmartDashboard.putNumber("AprilTagZ", getTranslation().getZ());
    }
  }
