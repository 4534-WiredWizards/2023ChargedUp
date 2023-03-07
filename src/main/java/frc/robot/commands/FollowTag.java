// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimator;

public class FollowTag extends CommandBase {
  /** Creates a new FollowTag. */
  private final PhotonCamera photoncamera;
  private final DriveSubsystem driveSubsystem;
  private PoseEstimator poseEstimator;
  private Pose2d robotPose2d;
  private Pose3d targetPose;
  private Pose3d robotPose;

  private final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(2, 1);
  private final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(2, 1);
  private final TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(0.5, 0.5);


  private final ProfiledPIDController xController = new ProfiledPIDController(1, 0, 0, xConstraints);
  private final ProfiledPIDController yController = new ProfiledPIDController(1, 0, 0, yConstraints);
  private final ProfiledPIDController rotationController = new ProfiledPIDController(0.5, 0, 0, rotationConstraints);

  private final Transform3d tagToGoal = new Transform3d(new Translation3d(1.5, 0, 0), new Rotation3d(0, 0, 0));


  public FollowTag(PoseEstimator pose, PhotonCamera camera, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    poseEstimator = pose;
    photoncamera = camera;
    driveSubsystem = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotPose2d = poseEstimator.getCurrentPose();
    robotPose = new Pose3d(robotPose2d.getX(), robotPose2d.getY(), 0.0, new Rotation3d(0.0, 0.0, driveSubsystem.getGyro()));
    targetPose = poseEstimator.getTargetPose();
    Pose2d goalPose = targetPose.transformBy(tagToGoal).toPose2d();
    xController.setGoal(goalPose.getX());
    yController.setGoal(goalPose.getY());
    rotationController.setGoal(goalPose.getRotation().getRadians());

    double xspeed = xController.calculate(robotPose2d.getX());
    double yspeed = yController.calculate(robotPose2d.getY());
    double rotationspeed = rotationController.calculate(robotPose2d.getRotation().getRadians());
    if (xController.atGoal()) {
      xspeed = 0;
    }
    if (yController.atGoal()) {
      yspeed = 0;
    }
    if (rotationController.atGoal()) {
      rotationspeed = 0;
    }
   
   
    driveSubsystem.drive(xspeed, yspeed, rotationspeed, true);
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
