package frc.robot.commands;

// Imports for the drivetarget command
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.DynamicSlewRateLimiter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.AprilTags;

public class DriveTargetCommand extends Command {
  private final double visionThingy = 1.25;

  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private CommandXboxController driverController;
  private double speedscalar = 1;


  private double forwardPWM;
  private double strafePWM;
  private double forwardVoltage;
  private double strafeVoltage;
  private double forwardLinearSpeed;
  private double strafeLinearSpeed;
  private double forwardAdjustedLinearSpeed;
  private double strafeAdjustedLinearSpeed;
  private double forwardAdjustedVoltage;
  private double strafeAdjustedVoltage;
  private double forwardAdjustedPWM;
  private double strafeAdjustedPWM;
  private double linearAccelerationLimit;
  private DynamicSlewRateLimiter x_rate = new DynamicSlewRateLimiter(DriveConstants.kMaxAcceleration);
  private DynamicSlewRateLimiter y_rate = new DynamicSlewRateLimiter(DriveConstants.kMaxAcceleration);
  private DynamicSlewRateLimiter theta_rate = new DynamicSlewRateLimiter(AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);
  private InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();

  private double forward = 0.0;
  private double strafe = 0.0;
  private double turn = 0.0;
  private double targetID = AprilTags.None.getId(); // Initialize targetID to None

  private final double desiredYaw = 0.0;
  private final double desiredRange = 0.75;
  private final double visionConstantAngle = 0.01;
  private final double visionConstantRange = 0.01;

  // Constructor
  public DriveTargetCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
      CommandXboxController driverController,
      ElevatorSubsystem elevatorSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.driverController = driverController;
    this.elevatorSubsystem = elevatorSubsystem;
    table.put(elevatorSubsystem.getMinHeight(), DriveConstants.kMaxAcceleration);
    table.put(elevatorSubsystem.getMaxHeight(), DriveConstants.kMinAcceleration);
    addRequirements(visionSubsystem, driveSubsystem);
    setTargetID(AprilTags.BlueProcessor);
  }

  public void setTargetID(AprilTags tagId) {
    this.targetID = tagId.getId(); // Set targetID based on the provided AprilTags enum value
  }

  // Executes the drivetarget command (periodic)
  @Override
  public void execute() {
    // forward =  x_rate.calculate(driverController.getLeftY() * speedscalar );
    // strafe =  y_rate.calculate(-driverController.getLeftX() * speedscalar );
    // turn = theta_rate.calculate(-0.3 * driverController.getRightX() * AutoConstants.kMaxAngularSpeedRadiansPerSecond);
    linearAccelerationLimit = table.get(elevatorSubsystem.getElevatorHeight());

    forwardPWM = driverController.getLeftY() * speedscalar;
    forwardVoltage = forwardPWM * DriveConstants.kDefaultBusVoltage;
    forwardLinearSpeed = forwardVoltage / DriveConstants.kForwardVoltsPerMeterPerSecond;
    forwardAdjustedLinearSpeed = x_rate.calculate(forwardLinearSpeed, linearAccelerationLimit);
    forwardAdjustedVoltage = forwardAdjustedLinearSpeed * DriveConstants.kForwardVoltsPerMeterPerSecond;
    forwardAdjustedPWM = forwardAdjustedVoltage / DriveConstants.kDefaultBusVoltage;

    strafePWM = -driverController.getLeftX() * speedscalar;
    strafeVoltage = strafePWM * DriveConstants.kDefaultBusVoltage;
    strafeLinearSpeed = strafeVoltage / DriveConstants.kStrafeVoltsPerMeterPerSecond;
    strafeAdjustedLinearSpeed = y_rate.calculate(strafeLinearSpeed, linearAccelerationLimit);
    strafeAdjustedVoltage = strafeAdjustedLinearSpeed * DriveConstants.kStrafeVoltsPerMeterPerSecond;
    strafeAdjustedPWM = strafeAdjustedVoltage / DriveConstants.kDefaultBusVoltage;
    
    forward = forwardAdjustedPWM;
    strafe = strafeAdjustedPWM;
    // forward =  (driverController.getLeftY() * speedscalar );
    // strafe =  (-driverController.getLeftX() * speedscalar );
    turn = theta_rate.calculate(-0.3 * driverController.getRightX() * AutoConstants.kMaxAngularSpeedRadiansPerSecond);
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
    DriveTargetCommandLogger();
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

  private void DriveTargetCommandLogger() {
    SmartDashboard.putNumber("Linear Acceleration Limit", linearAccelerationLimit);
    SmartDashboard.putNumber("Forward", forward);
    SmartDashboard.putNumber("Forward PWM", forwardPWM);
    SmartDashboard.putNumber("Forward Voltage", forwardVoltage);
    SmartDashboard.putNumber("Forward Linear Speed", forwardLinearSpeed);
    SmartDashboard.putNumber("Forward Adjusted Linear Speed", forwardAdjustedLinearSpeed);
    SmartDashboard.putNumber("Forward Adjusted Voltage", forwardAdjustedVoltage);
    SmartDashboard.putNumber("Forward Adjusted PWM", forwardAdjustedPWM);
    SmartDashboard.putNumber("Strafe", strafe);
    SmartDashboard.putNumber("Strafe PWM", strafePWM);
    SmartDashboard.putNumber("Forward Voltage", strafeVoltage);
    SmartDashboard.putNumber("strafe Linear Speed", strafeLinearSpeed);
    SmartDashboard.putNumber("strafe Adjusted Linear Speed", strafeAdjustedLinearSpeed);
    SmartDashboard.putNumber("strafe Adjusted Voltage", strafeAdjustedVoltage);
    SmartDashboard.putNumber("strafe Adjusted PWM", strafeAdjustedPWM);
  }
}
