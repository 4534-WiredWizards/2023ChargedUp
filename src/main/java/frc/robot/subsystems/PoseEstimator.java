// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class PoseEstimator extends SubsystemBase {

private static final List<Pose3d> targetPoses = List.of(
    new Pose3d(new Translation3d(3, 2, 0.5), new Rotation3d(0, 0, Math.PI)),
    new Pose3d(new Translation3d(0, 0, 0.5), new Rotation3d(0, 0, Math.PI))
);
private Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0, 0.5), new Rotation3d(0,0,0));

// private final Matrix<N3, N1> stateStdDevs = new VecBuilder(); 
// private final Vector<N5> localMeasurementStdDevs = new VecBuilder(); 
// private final Matrix<N3, N1> visionMeasurementStdDevs = new VecBuilder(); 

double camera_height = 5;
double target_height = 20;
double camera_pitch = 0.5;
double range;


SwerveDrivePoseEstimator poseEstimator;
Pose3d targetPose;

private double previousTimeStamp = 0;

Field2d field2d = new Field2d();
PhotonCamera camera = new PhotonCamera("photoncamera");
DriveSubsystem driveSubsystem;

  /** Creates a new PoseEstimator. */
  public PoseEstimator(PhotonCamera pcamera, DriveSubsystem drive) {
    camera = pcamera;
    driveSubsystem = drive;

    poseEstimator = new SwerveDrivePoseEstimator(
    DriveConstants.kinematics, 
    driveSubsystem.getHeading(), 
    driveSubsystem.getModulePositions(),
    new Pose2d()
    //stateStdDevs,
    //visionMeasurementStdDevs
    );
  //   poseEstimator = new SwerveDrivePoseEstimator(
  //     Nat.N7(),
  //     Nat.N7(),
  //     Nat.N5(),
  //     driveSubsystem.getGyro(),
  //     driveSubsystem.getModuleStates(),
  //     new Pose2d(),
  //     DriveConstants.kinematics,
  //     stateStdDevs,
  //     localMeasurementStdDevs,
  //     visionMeasurementStdDevs
  // );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = camera.getLatestResult();
    var timeStamp = result.getTimestampSeconds();
    if (timeStamp != previousTimeStamp && result.hasTargets()) {
        previousTimeStamp = timeStamp;
        PhotonTrackedTarget target = result.getBestTarget();
        int targetID = target.getFiducialId();
        if (target.getPoseAmbiguity() <= .2 && targetID >= 0 && targetID < targetPoses.size()) {
            range = PhotonUtils.calculateDistanceToTargetMeters(
                camera_height, 
                target_height,
                camera_pitch,
                target.getPitch()
                );
            targetPose = targetPoses.get(targetID);
            Transform3d camToTargetTrans = target.getBestCameraToTarget();
            Pose3d camPose = targetPose.transformBy(camToTargetTrans.inverse());
            Pose3d robotPose = camPose.transformBy(robotToCam.inverse());
            poseEstimator.addVisionMeasurement(robotPose.toPose2d(), timeStamp);
        }
    }
    field2d.setRobotPose(getCurrentPose());
}

public Pose2d getCurrentPose() {
  return poseEstimator.getEstimatedPosition();
}

public Pose3d getTargetPose() {
  return targetPose;
}

public double getDistance() {
  return range;
}

public void updateShuffleboard() {
  SmartDashboard.putNumber("Distance", getDistance());
  SmartDashboard.putNumber("Pose X", getCurrentPose().getX());
  SmartDashboard.putNumber("Pose Y", getCurrentPose().getY());
}

  
}

  
