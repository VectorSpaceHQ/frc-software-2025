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
  private final double visionThingy = 1.25;

  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private CommandXboxController driverController;
  private double targetID = 0;

  private SlewRateLimiter x_rate = new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  private SlewRateLimiter y_rate = new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  private SlewRateLimiter theta_rate = new SlewRateLimiter(AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);

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
   
    double forward =  x_rate.calculate(driverController.getLeftY() * AutoConstants.kMaxSpeedMetersPerSecond);
    double strafe =  y_rate.calculate(-driverController.getLeftX() * AutoConstants.kMaxSpeedMetersPerSecond);
    double turn = theta_rate.calculate(-0.3 * driverController.getRightX() * AutoConstants.kMaxAngularSpeedRadiansPerSecond);

    // Check if the camera is connected and displays the aiming and camera status
    if (visionSubsystem.isCameraConnected()) {
      SmartDashboard.putString("Aiming Status", "Camera Connected");

      // Check if target has been specified
      if (targetID != AprilTags.None.getId()) {
        // If the camera is connected, get the target yaw and drive towards it
        double targetYaw = visionSubsystem.getTargetYaw((int) targetID);
        double targetRange = visionSubsystem.getTargetRange((int) targetID);

        // Check if the target yaw is valid and displays the aiming status and yaw
        if (!Double.isNaN(targetYaw)) {
          turn = -targetYaw * 0.01 * AutoConstants.kMaxAngularSpeedRadiansPerSecond;
          SmartDashboard.putString("Aiming Status", "Aiming");
          SmartDashboard.putNumber("Target Yaw", targetYaw);
          // Reset the odometry if the estimated pose has targets
        }

        if (!Double.isNaN(targetRange)) {
          forward = (visionThingy - targetRange * 0.01 * AutoConstants.kMaxSpeedMetersPerSecond);
          SmartDashboard.putString("Aiming Status", "Aiming");
          SmartDashboard.putNumber("Target Yaw", targetYaw);
        }
      }
    } else {
      SmartDashboard.putString("Aiming Status", "Camera Not Connected");
    }

    driveSubsystem.drive(forward, strafe, turn, false);
  }

  // Ends the drivetarget command
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, true);
    SmartDashboard.putString("Aiming Status", "Command Ended");
  }
}
