package frc.robot.commands;

// Imports for the drivetarget command
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.AprilTags;

public class DriveTargetCommand extends Command {

  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final CommandXboxController driverController;

  private final SlewRateLimiter x_rate = new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  private final SlewRateLimiter y_rate = new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  private final SlewRateLimiter theta_rate = new SlewRateLimiter(AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);

  private double forward = 0.0;
  private double strafe = 0.0;
  private double turn = 0.0;
  private double targetID = AprilTags.None.getId(); // Initialize targetID to None

  private final double desiredYaw = 0.0;
  private final double desiredRange = 0.75;
  private final double visionConstantAngle = 0.01;
  private final double visionConstantRange = 0.01;
  private double speedscalar = 0.1;

  // Constructor
  public DriveTargetCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
      CommandXboxController driverController) {
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.driverController = driverController;
    addRequirements(visionSubsystem, driveSubsystem);
    setTargetID(AprilTags.BlueProcessor);
  }

  public void setTargetID(AprilTags tagId) {
    this.targetID = tagId.getId(); // Set targetID based on the provided AprilTags enum value
  }

  // Executes the drivetarget command (periodic)
  @Override
  public void execute() {
    // Initialize forward, strafe, and turn to zero
    forward = 0.0;
    strafe = 0.0;
    turn = 0.0;

    // Check if the camera is connected and displays the aiming and camera status
    if (visionSubsystem.isCameraConnected()) {
      SmartDashboard.putString("Aiming Status", "Camera Connected");

      // Check if target has been specified
      if (targetID != AprilTags.None.getId()) {
        // If the camera is connected, get the target yaw and drive towards it
        double targetYaw = visionSubsystem.getTargetYaw((int) targetID);
        double tagRange = visionSubsystem.getTargetRange((int) targetID);
        SmartDashboard.putNumber("Target ID", targetID);

        // Check if the target yaw is valid and displays the aiming status and yaw
        if (!Double.isNaN(targetYaw)) {
          turn = ((desiredYaw - targetYaw) * visionConstantAngle * AutoConstants.kMaxAngularSpeedRadiansPerSecond);
          SmartDashboard.putString("Aiming Status", "Aiming");
          SmartDashboard.putNumber("Target Yaw", targetYaw);
          
        } else {
          SmartDashboard.putString("Aiming Status", "No Valid Target");
        }

        // Check if the target range is valid and update forward speed
        if (!Double.isNaN(tagRange)) {
          forward = ((tagRange - desiredRange) * visionConstantRange * AutoConstants.kMaxSpeedMetersPerSecond);
          SmartDashboard.putString("Aiming Status", "Driving Forward");
          SmartDashboard.putNumber("Target Range", tagRange);

        }
        if (!Double.isNaN(targetYaw) && !Double.isNaN(tagRange)) {
          SmartDashboard.putString("Aiming Status", "Aiming and Driving Forward");
        }

      } else {
        SmartDashboard.putString("Aiming Status", "No Target Specified");
      }
    } else {
      SmartDashboard.putString("Aiming Status", "Camera Not Connected");
    }

    // If no valid target ID, use driver inputs
    if (targetID == AprilTags.None.getId()) {
      forward = x_rate.calculate(driverController.getLeftY() * 0.5);
      strafe = y_rate.calculate(-driverController.getLeftX() * speedscalar);
      turn = theta_rate.calculate(-0.3 * driverController.getRightX() * AutoConstants.kMaxAngularSpeedRadiansPerSecond);
    }

    // Drive robot with values calculated above
    driveSubsystem.drive(forward, strafe, turn, true);
  }

  // Ends the drivetarget command
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false);
    SmartDashboard.putString("Aiming Status", "Command Ended");
  }

  public void setSpeedScalar(double val) {
    speedscalar = val;
  }
}