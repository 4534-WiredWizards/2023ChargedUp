package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class QuickTurn extends CommandBase {

    /**
     * A command to align the robot automatically to a certain heading, used for a "quick turn" for shooting
     */

    private final DriveSubsystem drive;

    private final double desiredAngle;
    private double currentAngle;
    private double oldAngle;
    private double tolerance = 1;

    private final PIDController controller = new PIDController(
        1.0, 0, 0
    );

    public QuickTurn(DriveSubsystem subsystem, double desiredAngleRad) {

        drive = subsystem;

        desiredAngle = desiredAngleRad;

        oldAngle = drive.getGyro();

    }

    @Override
    public void execute() {
            
        //calculate the next output for the drive based on the current heading, setpoint being the given angle
        double rotationOut = controller.calculate(drive.getHeading().getRadians(), desiredAngle);

        //allows for driver control for strafing but takes control of the rotation output
        drive.drive(
            drive.getCommandedDriveValues()[0], 
            drive.getCommandedDriveValues()[1], 
            rotationOut, 
            drive.getIsFieldRelative());
        
        currentAngle = drive.getGyro();

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Quickturn end");
        drive.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        //all angles are in degrees
        if (currentAngle <= oldAngle + Math.toDegrees(desiredAngle) + tolerance && currentAngle >= oldAngle + Math.toDegrees(desiredAngle) - tolerance) {
            
            System.out.println("Current Angle: " + currentAngle + "   Old Angle: " + oldAngle + "   Desired Angle: " + Math.toDegrees(desiredAngle));
            return true;
        }
        else {
            System.out.println("Current Angle: " + currentAngle + "   Old Angle: " + oldAngle + "   Desired Angle: " + Math.toDegrees(desiredAngle));
            return false;
        }
    }

   

}
