package frc.robot.commands;

// Imports for the drivetarget command
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.AprilTags;
import frc.robot.Constants;

public class DriveTargetCommand extends Command {
  private final double desiredRange = 1.25;

  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private CommandXboxController driverController;
  private double speedscalar = 0.1;
  private double targetID = 0;

  private SlewRateLimiter x_rate = new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  private SlewRateLimiter y_rate = new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  private SlewRateLimiter theta_rate = new SlewRateLimiter(AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);

  double forward = 0;
  double strafe = 0;
  double turn = 0;

  // Sets the drivetarget constructor
  public DriveTargetCommand(
      DriveSubsystem driveSubsystem,
      VisionSubsystem visionSubsystem,
      CommandXboxController driverController) {
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.driverController = driverController;
    addRequirements(visionSubsystem, driveSubsystem);
    setTargetID(AprilTags.None);
  }

  public void setTargetID(AprilTags tagId) {
    targetID = tagId.getId();
  }

  // Executes the drivetarget command (periodic)
  @Override
  public void execute() {

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
          turn = -targetYaw * 0.01 * AutoConstants.kMaxAngularSpeedRadiansPerSecond;
          SmartDashboard.putString("Aiming Status", "Aiming");
          SmartDashboard.putNumber("Target Yaw", targetYaw);
        }

        if (!Double.isNaN(tagRange)) {
          forward = ((tagRange - desiredRange) * 0.01 * AutoConstants.kMaxSpeedMetersPerSecond);
          SmartDashboard.putString("Aiming Status", "Driving Forward");
          SmartDashboard.putNumber("Target Range", tagRange);
        }

      } else {
        SmartDashboard.putString("Aiming Status", "No Target Specified");
      }

    } else {
      SmartDashboard.putString("Aiming Status", "Camera Not Connected");
    }

    // Driver controls if NO april tag is identified
    if (targetID == AprilTags.None.getId()) {
      forward = x_rate.calculate(driverController.getLeftY() * 0.5);
      strafe = y_rate.calculate(-driverController.getLeftX() * speedscalar);
      turn = theta_rate.calculate(-0.3 * driverController.getRightX() * AutoConstants.kMaxAngularSpeedRadiansPerSecond);
      SmartDashboard.putString("Aiming Status", "No Target Specified");
    }

  driveSubsystem.drive(forward, strafe, turn, false);
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
