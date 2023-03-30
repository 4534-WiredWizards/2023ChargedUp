package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;


public class Constants {

    public static final class CANDevices {

        public static final int frontLeftRotationMotorId = 41; //11
        public static final int frontLeftDriveMotorId = 40; //10

        public static final int frontRightRotationMotorId = 31; //21
        public static final int frontRightDriveMotorId =30; //20

        public static final int rearLeftRotationMotorId = 21; //31
        public static final int rearLeftDriveMotorId = 20; //30

        public static final int rearRightRotationMotorId = 11; //41
        public static final int rearRightDriveMotorId = 10; //40

        //public static final int rightClimberMotorId = 29;
        //public static final int leftClimberMotorId = 19;

        public static final int frontLeftRotationEncoderId = 42; //12
        public static final int frontRightRotationEncoderId = 32; //22
        public static final int rearLeftRotationEncoderId = 22; //32
        public static final int rearRightRotationEncoderId = 12; //42

        public static final int armMotorOne = 15;
        public static final int armEncoder = 37;

        //public static final int intakeMotorOne = 16;
        //public static final int intakeMotorTwo = 26;


        // Using values from can encoders from shuffleboard to calculate offsets
        public static final double frontLeftAngleOffset = Units.degreesToRadians(181.4); //1.7 //180.7
        public static final double frontRightAngleOffset = Units.degreesToRadians(128.5);  //90 

        public static final double rearLeftAngleOffset = Units.degreesToRadians(265.6);  //84.6 //263.5 //EFFECTS Front Right
        public static final double rearRightAngleOffset = Units.degreesToRadians(16.61); //61.6 //240.7
    
        //public static final int feederWheelMotorId = 35;
        // public static final int rightFlywheelMotorId = 25;
        // public static final int leftFlywheelMotorId = 15;

        //public static final int hoodMotorId = 45;

        // public static final int leftIntakeMotorId = 26;  // cAUSING ISSUES WITH MULTIPLE INSTACES AND LOW SPEED?!?
        // public static final int rightIntakeMotorId = 16;
        // public static final int centerIntakeMotorId = 36;

        public static final int imuId = 18;

        public static final double reductionFactor = 1/1;

    }

    public static final class DIOChannels {

        public static final int bottomBannerPort = 0;
        public static final int topBannerPort = 9;

    }

    public static final class InputDevices {

        public static final int leftJoystickPort = 0;
        public static final int rightJoystickPort = 1;

        public static final int gamepadPort = 2;

        public static final int btn_a = 1;
        public static final int btn_b = 2;
        public static final int btn_x = 3;
        public static final int btn_y = 4;
        public static final int btn_leftBumper = 5;
        public static final int btn_rightBumper = 6;
        public static final int btn_leftTrigger = 2;
        public static final int btn_rightTrigger = 3;
        public static final int btn_select = 7;
        public static final int btn_start = 8;
    }

    public static final class fancyJoystick {

        public static final int leftJoystickPort = 0;
        public static final int rightJoystickPort = 1;

        public static final int r1 = 1; 
        public static final int l1 = 2; 
        public static final int r3 = 3;
        public static final int l3 = 4; 
        public static final int square = 5;
        public static final int cross = 6;
        public static final int circle = 7;
        public static final int triangle = 8;
        public static final int r2 = 9;
        public static final int l2 = 10;

        public static final int seesaw = 4;
        public static final int leftControl = 2;
        public static final int rightY = 1;
        public static final int rightX = 0;
        public static final int rotate = 3;
        public static final int se = 11;
        public static final int st = 12; 


    }

    public static final class SpeedConstants {
        public static final double shooterSpeed = 0.65;
        public static final double feederWheelSpeed = 0.6; 
        public static final double intakeSpeed = 0.5;
        public static final double climbUpSpeed = 1;
        public static final double climbDownSpeed = 0.75;
    }

    public static final class PneumaticChannels {

        public  static final int PCMId = 1;

        public static final int leftIntakeSolenoidChannel = 3;  //0
        public static final int rightIntakeSolenoidChannel = 4; //1
        public static final int centerIntakeSolenoidChannel = 5; //2

        public static final int deploySolenoidChannel = 15;
        public static final int retractSolenoidChannel = 8;

    }

    public static final class DriveConstants {

        public static final double trackWidth = Units.inchesToMeters(20.5); //22 //17.25 //width between left and right swerve moudles
        public static final double wheelBase = Units.inchesToMeters(16.5); //22 //28.5 //length between front and back motors

        public static final SwerveDriveKinematics kinematics = 
            new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0), //front left
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), //front right
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), //rear left
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) //rear right
            );
            

        public static final double driveWheelGearReduction = 6.75; //6.86 for MK3
        public static final double rotationWheelGearReduction = 12.8; //12.8 for MK3

        public static final double wheelDiameterMeters = 0.050686 * 2;

        public static final double rotationMotorMaxSpeedRadPerSec = 1.0;
        public static final double rotationMotorMaxAccelRadPerSecSq = 1.0;

        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.254, 0.137);

        public static final double maxDriveSpeed = 12.0;
        public static final double teleopTurnRateDegPerSec = 240; //120 //360.0;  was at 90 but we lowered because it was very bumpy
                                                                 // 45 worked but was too slow :(    //Rate the robot will spin with full rotation command
        
        //speedScaleSlope is the m section of y=mx+b, speedScaleOffset is b
                                                                 
        public static final double speedScaleSlope = (-0.25);
        public static final double speedScaleOffset = (0.75);

    }

    public static final class SuperstructureConstants {

        public static final double baseShootingSpeed = 6000; //rptations per minute

        public static final double intakingPower = 0.4;
        public static final double jogFowardPower = 0.2;

        public static final double kickerPower = 1.0;
        public static final double conveyorPower = 0.35;

        public static final double flywheelGearRatio = 32.0 / 18.0;

        public static final double jogDelaySeconds = 0.1;

    }

    public static final class ClimbingConstants {

        public static final double climberMotorGearReduction = 1;

        public static final double winchDiameterInches = 1;
        public static final double climberMaxHeightInches = 40.5;

        public static final double desiredClimberSpeedInchesPerSecond = 1;

    }

    public static final class HoodConstants {
        public static final double normalHoodSpeed = 0.2; 
        public static final double slowHoodSpeed = 0.05;
        public static final double low = 1;
        public static final double high = 2;
        public static final double far = 3;
        public static final double veryfar = 4;
        public static final double lowPosition = 0.7;
        public static final double highPosition = 0.16; 
        public static final double farPosition = 0.95; 
        public static final double veryfarPosition = 0.71; //1.05
        public static final double lowShooterSpeed = 0.31;
        public static final double highShooterSpeed = 0.6; //0.6
        public static final double farShooterSpeed = 0.67;
        public static final double veryfarShooterSpeed = 0.6;  //0.77
    }

    public static final class VisionConstants {

        public static final double limelightHeightInches = 26.5; // distance from limelight to ground
        public static final double limelightMountAngleRadians = Units.degreesToRadians(53);

    }

    public static final class AutoConstants {

        public static final double maxVelMetersPerSec = 3; //3; //2
        public static final double maxAccelMetersPerSecondSq = 1; //1 //1.95
        public static final double positionOneShootingAngle = 90;
        public static final double positionTwoShootingAngle = 90;

        public static final int leftIntake = 0;
        public static final int centerIntake = 1;
        public static final int rightIntake = 2;
        
    }

    public static final class FieldConstants {

        public static final double targetHeightInches = 89.5;

    }

    public static final class VelocityClosedLoop {
            /**
         * Which PID slot to pull gains from. Starting 2018, you can choose from
         * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
         * configuration.
         */
        public static final int kSlotIdx = 0;

        /**
         * Talon FX supports multiple (cascaded) PID loops. For
         * now we just want the primary one.
         */
        public static final int kPIDLoopIdx = 0;

        /**
         * Set to zero to skip waiting for confirmation, set to nonzero to wait and
         * report to DS if action fails.
         */
        public static final int kTimeoutMs = 30;

        /**
         * PID Gains may have to be adjusted based on the responsiveness of control loop.
         * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
         * 
         * 	                                    			  kP   	 kI    kD      kF          Iz    PeakOut */
        public final static Gains kGains_Velocit  = new Gains( 0.1, 0.001, 5, 1023.0/21777.0,  300,  1.00);

        public final static double maxRPM = 21777;
    }
    
    
}
