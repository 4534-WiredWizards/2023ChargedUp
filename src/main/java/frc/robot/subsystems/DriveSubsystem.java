package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;
import com.kauailabs.navx.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {

    /**
     * Subsystem that controls the drivetrain of the robot
     * Handles all the odometry and base movement for the chassis
     */

    /**
     * absolute encoder offsets for the wheels
     * 180 degrees added to offset values to invert one side of the robot so that it doesn't spin in place
     */
    //private static final double frontLeftAngleOffset = Units.degreesToRadians(22.0);
    //private static final double frontRightAngleOffset = Units.degreesToRadians(114.34);
    //private static final double rearLeftAngleOffset = Units.degreesToRadians(109.1);
    //private static final double rearRightAngleOffset = Units.degreesToRadians(37.7);

    /**
     * SwerveModule objects
     * Parameters:
     * drive motor can ID
     * rotation motor can ID
     * external CANCoder can ID
     * measured CANCoder offset
     */

    private final SwerveModule frontLeft = 
        new SwerveModule(
            CANDevices.frontLeftDriveMotorId,
            CANDevices.frontLeftRotationMotorId,
            CANDevices.frontLeftRotationEncoderId,
            CANDevices.frontLeftAngleOffset
        );

    private final SwerveModule frontRight = 
        new SwerveModule(
            CANDevices.frontRightDriveMotorId,
            CANDevices.frontRightRotationMotorId,
            CANDevices.frontRightRotationEncoderId,
            CANDevices.frontRightAngleOffset
        );

    private final SwerveModule rearLeft = 
        new SwerveModule(
            CANDevices.rearLeftDriveMotorId,
            CANDevices.rearLeftRotationMotorId,
            CANDevices.rearLeftRotationEncoderId,
            CANDevices.rearLeftAngleOffset
        );

    private final SwerveModule rearRight = 
        new SwerveModule(
            CANDevices.rearRightDriveMotorId,
            CANDevices.rearRightRotationMotorId,
            CANDevices.rearRightRotationEncoderId,
            CANDevices.rearRightAngleOffset
        );

        

    // commanded values from the joysticks and field relative value to use in AlignWithTargetVision and AlignWithGyro
    private double commandedForward = 0;
    private double commandedStrafe = 0;
    private double commandedRotation = 0;

    private boolean isCommandedFieldRelative = true;

    //private final PigeonIMU imu = new PigeonIMU(CANDevices.imuId);
    public AHRS ahrs = new AHRS();

    private final ADXRS450_Gyro spi_gyro = new ADXRS450_Gyro();

    /**
     * odometry for the robot, measured in meters for linear motion and radians for rotational motion
     * Takes in kinematics and robot angle for parameters
     */
    private final SwerveDriveOdometry odometry = 
        new SwerveDriveOdometry(
            DriveConstants.kinematics, 
            new Rotation2d(getHeading().getRadians()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition(),
            }
        );

    public DriveSubsystem() {

        ahrs.zeroYaw();

        // initialize the rotation offsets for the CANCoders
        frontLeft.initRotationOffset();
        frontRight.initRotationOffset();
        rearLeft.initRotationOffset();
        rearRight.initRotationOffset();

        // reset the measured distance driven for each module
        frontLeft.resetDistance();
        frontRight.resetDistance();
        rearLeft.resetDistance();
        rearRight.resetDistance();

    }

    @Override
    public void periodic() {

        // update the odometry every 20ms
        odometry.update(getHeading(), getModulePositions());
        SmartDashboard.putNumber("odometry angle",odometry.getPoseMeters().getRotation().getDegrees());
    }
    
    /**
     * method for driving the robot
     * Parameters:
     * forward linear value
     * sideways linear value
     * rotation value
     * if the control is field relative or robot relative
     * @return 
     */
    double forward, strafe, rotation;

    public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

        // update the drive inputs for use in AlignWithGyro and AlignWithTargetVision control
        this.forward += Math.max(-0.003, Math.min(forward - this.forward, 0.003));
        this.rotation += Math.max(-0.003, Math.min(rotation - this.rotation, 0.003));
        this.strafe += Math.max(-0.1, Math.min(strafe - this.strafe, 0.1));
        commandedForward = this.forward;
        commandedStrafe = strafe;
        commandedRotation = rotation;

        isCommandedFieldRelative = isFieldRelative;

        /**
         * ChassisSpeeds object to represent the overall state of the robot
         * ChassisSpeeds takes a forward and sideways linear value and a rotational value
         * 
         * speeds is set to field relative or default (robot relative) based on parameter
         */
        ChassisSpeeds speeds =
            isCommandedFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    forward, strafe, rotation, getHeading()) //Inverted rotation during field relative driving
                : new ChassisSpeeds(forward, strafe, rotation);
        
        // use kinematics (wheel placements) to convert overall robot state to array of individual module states
        SwerveModuleState[] states = DriveConstants.kinematics.toSwerveModuleStates(speeds);

        states[0] = SwerveModuleState.optimize(states[0], frontLeft.getCanEncoderAngle());
        states[1] = SwerveModuleState.optimize(states[1], frontRight.getCanEncoderAngle());
        states[2] = SwerveModuleState.optimize(states[2], rearLeft.getCanEncoderAngle());
        states[3] = SwerveModuleState.optimize(states[3], rearRight.getCanEncoderAngle());

        // make sure the wheels don't try to spin faster than the maximum speed possible
        //SwerveDriveKinematics.normalizeWheelSpeeds(states, DriveConstants.maxDriveSpeed);

        setModuleStates(states);

    }

    /**
     * Method to set the desired state for each swerve module
     * Uses PID and feedforward control to control the linear and rotational values for the modules
     */
    public void setModuleStates(SwerveModuleState[] moduleStates) {

        frontLeft.setDesiredStateClosedLoop(moduleStates[0]);
        frontRight.setDesiredStateClosedLoop(moduleStates[1]);
        rearLeft.setDesiredStateClosedLoop(moduleStates[2]);
        rearRight.setDesiredStateClosedLoop(moduleStates[3]);

    }

    // returns an array of SwerveModuleState
    public SwerveModuleState[] getModuleStates() {

        SwerveModuleState[] states = {
            new SwerveModuleState(frontLeft.getCurrentVelocityMetersPerSecond(), frontLeft.getCanEncoderAngle()),
            new SwerveModuleState(frontRight.getCurrentVelocityMetersPerSecond(), frontRight.getCanEncoderAngle()),
            new SwerveModuleState(rearLeft.getCurrentVelocityMetersPerSecond(), rearLeft.getCanEncoderAngle()),
            new SwerveModuleState(rearRight.getCurrentVelocityMetersPerSecond(), rearRight.getCanEncoderAngle())
        };

        return states;

    }

    public SwerveModulePosition[] getModulePositions() {

        SwerveModulePosition[] positions = {
            new SwerveModulePosition(frontLeft.getDriveDistanceRadians() * (DriveConstants.wheelDiameterMeters / 2.0), frontLeft.getCanEncoderAngle()),
            new SwerveModulePosition(frontRight.getDriveDistanceRadians() * (DriveConstants.wheelDiameterMeters / 2.0), frontRight.getCanEncoderAngle()),
            new SwerveModulePosition(rearLeft.getDriveDistanceRadians() * (DriveConstants.wheelDiameterMeters / 2.0), rearLeft.getCanEncoderAngle()),
            new SwerveModulePosition(rearRight.getDriveDistanceRadians() * (DriveConstants.wheelDiameterMeters / 2.0), rearRight.getCanEncoderAngle())
        };

        return positions;

    }


    /**
     * Return the current position of the robot on field
     * Based on drive encoder and gyro reading
     */
    public Pose2d getPose() {

        return odometry.getPoseMeters();

    }

    // reset the current pose to a desired pose
    public void resetPose(Pose2d pose) {

        ahrs.zeroYaw();
        odometry.resetPosition(getHeading(), getModulePositions(), pose);

    }

    // reset the measured distance driven for each module
    public void resetDriveDistances() {

        frontLeft.resetDistance();
        frontRight.resetDistance();
        rearLeft.resetDistance();
        rearRight.resetDistance();

    }

    // return the average distance driven for each module to get an overall distance driven by the robot
    public double getAverageDriveDistanceRadians() {

        return ((
            Math.abs(frontLeft.getDriveDistanceRadians())
            + Math.abs(frontRight.getDriveDistanceRadians())
            + Math.abs(rearLeft.getDriveDistanceRadians())
            + Math.abs(rearRight.getDriveDistanceRadians())) / 4.0);

    }

    // return the average velocity for each module to get an overall velocity for the robot
    public double getAverageDriveVelocityRadiansPerSecond() {

        return ((
            Math.abs(frontLeft.getCurrentVelocityRadiansPerSecond())
            + Math.abs(frontRight.getCurrentVelocityRadiansPerSecond()) 
            + Math.abs(rearLeft.getCurrentVelocityRadiansPerSecond()) 
            + Math.abs(rearRight.getCurrentVelocityRadiansPerSecond())) / 4.0);

    }

    // get the current heading of the robot based on the gyro
    public Rotation2d getHeading() {
        
        double[] ypr = new double[3];
        ypr[0] = 0-ahrs.getAngle();
        SmartDashboard.putNumber("ypr(angle)", ypr[0]);
        SmartDashboard.putNumber("new gyro angle", spi_gyro.getAngle());
        
        return Rotation2d.fromDegrees(ypr[0]);



        //Jason Test Code
        // double[] rotation_2 = new double[3];
        // rotation_2[0] = ypr[0] - 180;

        // double place_holder_DesiredRotation = ypr[0];

        // double[] final_rotation = new double[3];

        // if (Math.abs(ypr[0] - place_holder_DesiredRotation) < Math.abs(rotation_2[0] = ypr[0] - place_holder_DesiredRotation)) {
        //     final_rotation[0] = ypr[0];
        // }
        // else {
        //     final_rotation[0] = rotation_2[0];
        // }
        // return Rotation2d.fromDegrees(final_rotation[0] + 180);

    }

    public double[] getCommandedDriveValues() {

        double[] values = {commandedForward, commandedStrafe, commandedRotation};

        return values;

    }

    public boolean getIsFieldRelative() {

        return isCommandedFieldRelative;

    }

    public void setFieldRelative() {
        isCommandedFieldRelative = !isCommandedFieldRelative;
    }

    public double getGyro() {
        return -ahrs.getAngle();
    }

    public double getPitch() {
        return ahrs.getPitch();
    }

    public double relativeDriveDirection() {
        double angle = Math.toDegrees(Math.atan2(commandedForward, -commandedStrafe));
        if(angle < 0) {
            return angle + 360;
        }
        return angle;
    }

    public void resetImu() {

        ahrs.zeroYaw();
    }

    
    public void updateSmartDashboard(){
        
    SmartDashboard.putNumber("Front Left Angle: ", frontLeft.getCanCoderRawAngle());
    SmartDashboard.putNumber("Front Right Angle: ", frontRight.getCanCoderRawAngle());
    SmartDashboard.putNumber("Back Left Angle: ", rearLeft.getCanCoderRawAngle());
    SmartDashboard.putNumber("Back Right Angle: ", rearRight.getCanCoderRawAngle());
    SmartDashboard.putNumber("Gyro Angle", -ahrs.getAngle());
    SmartDashboard.putNumber("Gyro Pitch", -ahrs.getPitch());

    SmartDashboard.putNumber("Front Left Adjusted Angle: ", frontLeft.getCanCoderAngle().getDegrees());
    SmartDashboard.putNumber("Front Right Adjusted Angle: ", frontRight.getCanCoderAngle().getDegrees());
    SmartDashboard.putNumber("Back Left Adjusted Angle: ", rearLeft.getCanCoderAngle().getDegrees());
    SmartDashboard.putNumber("Back Right Adjusted Angle: ", rearRight.getCanCoderAngle().getDegrees());
    SmartDashboard.putNumber("Front Left NEO: ", frontLeft.getCanEncoderAngle().getDegrees());
    SmartDashboard.putNumber("Front Right NEO: ", frontRight.getCanEncoderAngle().getDegrees());
    SmartDashboard.putNumber("Rear Left NEO: ", rearLeft.getCanEncoderAngle().getDegrees());
    SmartDashboard.putNumber("Rear Right NEO: ", rearRight.getCanEncoderAngle().getDegrees());



    
    }

}