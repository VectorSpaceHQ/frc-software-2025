package frc.robot.commands;

// Imports for the drivetarget command
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveTargetCommand extends Command {
    private DriveSubsystem driveSubsystem;
    private VisionSubsystem visionSubsystem;
    private XboxController driverController;

    // Sets the drivetarget constructor
    public DriveTargetCommand(
            DriveSubsystem driveSubsystem,
            VisionSubsystem visionSubsystem,
            XboxController driverController) {
        this.visionSubsystem = visionSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.driverController = driverController;
        addRequirements(visionSubsystem, driveSubsystem);
    }

    // Executes the drivetarget command (periodic)
    @Override
    public void execute() {
        double forward = -driverController.getLeftY() * AutoConstants.kMaxSpeedMetersPerSecond;
        double strafe = -driverController.getRightX() * AutoConstants.kMaxSpeedMetersPerSecond;
        double turn = -driverController.getLeftX() * AutoConstants.kMaxAngularSpeedRadiansPerSecond;

        // Check if the camera is connected and displays the aiming and camera status
        if (visionSubsystem.isCameraConnected()) {
            SmartDashboard.putString("Aiming Status", "Camera Connected");

            // Check if the A button is pressed
            if (driverController.getAButton()) {
                // If the camera is connected, get the target yaw and drive towards it
                double targetYaw = visionSubsystem.getTargetYaw(1);

                // Check if the target yaw is valid and displays the aiming status and yaw
                if (!Double.isNaN(targetYaw)) {
                    turn = -targetYaw * 0.01 * AutoConstants.kMaxAngularSpeedRadiansPerSecond;
                    SmartDashboard.putString("Aiming Status", "Aiming");
                    SmartDashboard.putNumber("Target Yaw", targetYaw);
                    // Reset the odometry if the estimated pose has targets
                }
            }
        }
        else {
            SmartDashboard.putString("Aiming Status", "Camera Not Connected");
        }

        driveSubsystem.drive(forward, strafe, turn, true);
    }

    // Ends the drivetarget command
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, true);
        SmartDashboard.putString("Aiming Status", "Command Ended");
    }
}
