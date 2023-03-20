package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.InputDevices;
import frc.robot.Constants.fancyJoystick;
import frc.robot.subsystems.DriveSubsystem;

public class OperatorControl extends CommandBase {

    /**
     * Command to allow for driver input in teleop
     * Can't be inlined efficiently if we want to edit the inputs in any way (deadband, square, etc.)
     */

    private final DriveSubsystem drive;

    /**
     * Joysticks return DoubleSuppliers when the get methods are called
     * This is so that joystick getter methods can be passed in as a parameter but will continuously update, 
     * versus using a double which would only update when the constructor is called
     */
    private final DoubleSupplier forwardX;
    private final DoubleSupplier forwardY;
    private final DoubleSupplier rotation;
    private final DoubleSupplier speedAdjust;
    
    private final boolean isFieldRelative;
    private double rotationConstant = 1;

    public OperatorControl(
        DriveSubsystem subsystem, 
        DoubleSupplier fwdX, 
        DoubleSupplier fwdY, 
        DoubleSupplier rot,
        DoubleSupplier throttle,
        boolean fieldRelative
    ) {

        drive = subsystem;
        forwardX = fwdX;
        forwardY = fwdY;
        rotation = rot;
        speedAdjust = throttle;

        isFieldRelative = fieldRelative;

        
        addRequirements(subsystem);

    }
    
    @Override
    public void execute() {

        /**
         * Units are given in meters per second radians per second
         * Since joysticks give output from -1 to 1, we multiply the outputs by the max speed
         * Otherwise, our max speed would be 1 meter per second and 1 radian per second
         */
        if (frc.robot.RobotContainer.m_fancyJoystick.getRawButton(fancyJoystick.r1)) {
            rotationConstant = 0.2;
         }
         else {
            rotationConstant = 1;
         }

        double speedScale = speedAdjust.getAsDouble()*DriveConstants.speedScaleSlope+DriveConstants.speedScaleOffset;

        // All inputs inverted because initial state is pointed towards drivers!
        
        double fwdX = -1 * forwardX.getAsDouble();
        fwdX = Math.copySign(fwdX, fwdX);
        fwdX = deadbandInputs(fwdX) * Units.feetToMeters(DriveConstants.maxDriveSpeed) * speedScale;

        double fwdY = -1 * forwardY.getAsDouble();
        fwdY = Math.copySign(fwdY, fwdY);
        fwdY = deadbandInputs(fwdY) * Units.feetToMeters(DriveConstants.maxDriveSpeed) * speedScale;

        double rot = Math.pow(-1 * rotation.getAsDouble(), 3);
        rot = Math.copySign(rot * rot, rot);
        rot = deadbandInputs(rot) * Units.degreesToRadians(DriveConstants.teleopTurnRateDegPerSec*rotationConstant);
        


        drive.drive(
            -fwdX,
            -fwdY,
            -rot,
            isFieldRelative
        );

    }

    // method to deadband inputs to eliminate tiny unwanted values from the joysticks
    public double deadbandInputs(double input) {

        if (Math.abs(input) < 0.1 /*0.035*/) return 0.0;
        return input;

    }

}