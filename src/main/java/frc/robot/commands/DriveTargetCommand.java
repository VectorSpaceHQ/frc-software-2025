package frc.robot.commands;

// Imports for the drivetarget command
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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

  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private CommandXboxController driverController;
  private double speedscalar = 1;

  private double forwardPWM;
  private double turnPWM;
  private double strafePWM;
  private double forwardVoltage;
  private double strafeVoltage;
  private double turnVoltage;
  private double forwardLinearSpeed;
  private double strafeLinearSpeed;
  private double turnAngularSpeed;
  private double forwardAdjustedLinearSpeed;
  private double strafeAdjustedLinearSpeed;
  private double turnAdjustedAngularSpeed;
  private double forwardAdjustedVoltage;
  private double strafeAdjustedVoltage;
  private double turnAdjustedVoltage;
  private double forwardAdjustedPWM;
  private double strafeAdjustedPWM;
  private double turnAdjustedPWM;

  private double linearAccelerationLimit;
  private DynamicSlewRateLimiter x_rate = new DynamicSlewRateLimiter(DriveConstants.kMaxAcceleration);
  private DynamicSlewRateLimiter y_rate = new DynamicSlewRateLimiter(DriveConstants.kMaxAcceleration);
  private SlewRateLimiter theta_rate = new SlewRateLimiter(
      AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared * (180 / Math.PI));
  private InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();

  private double forward = 0.0;
  private double strafe = 0.0;
  private double turn = 0.0;
  private double targetYaw = 0;
  private double tagRange = 0;
  private double targetID = AprilTags.None.getId(); // Initialize targetID to None
  private PIDController forwardPIDController = new PIDController(0.01, 0, 0);
  private PIDController strafePIDController = new PIDController(0.01, 0, 0);
  private PIDController turnPIDController = new PIDController(1, 0, 0);

  private final double desiredYaw = 0.0;
  private final double desiredYOffset = 0.3;
  private final double desiredXOffset = 0.0;

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
    forwardPIDController.setTolerance(0.2);
    strafePIDController.setTolerance(0.2);
  }

  public void setTargetID(AprilTags tagId) {
    this.targetID = tagId.getId(); // Set targetID based on the provided AprilTags enum value
  }

  // Executes the drivetarget command (periodic)
  @Override
  public void execute() {
    // forward = x_rate.calculate(driverController.getLeftY() * speedscalar );
    // strafe = y_rate.calculate(-driverController.getLeftX() * speedscalar );
    // turn = theta_rate.calculate(-0.3 * driverController.getRightX() *
    // AutoConstants.kMaxAngularSpeedRadiansPerSecond);
    linearAccelerationLimit = table.get(elevatorSubsystem.getElevatorHeight());

    forwardPWM = -driverController.getLeftY() * speedscalar;
    forwardVoltage = forwardPWM * DriveConstants.kDefaultBusVoltage;
    forwardLinearSpeed = forwardVoltage / DriveConstants.kForwardVoltsPerMeterPerSecond;

    strafePWM = -driverController.getLeftX() * speedscalar;
    strafeVoltage = strafePWM * DriveConstants.kDefaultBusVoltage;
    strafeLinearSpeed = strafeVoltage / DriveConstants.kStrafeVoltsPerMeterPerSecond;

    // forward = (driverController.getLeftY() * speedscalar );
    // strafe = (-driverController.getLeftX() * speedscalar );
    turnPWM = -0.3 * driverController.getRightX() * AutoConstants.kMaxAngularSpeedRadiansPerSecond;
    turnVoltage = turnPWM * DriveConstants.kDefaultBusVoltage;
    turnAngularSpeed = turnVoltage / DriveConstants.kVoltsPerDegreePerSecond;

    // Check if the camera is connected and displays the aiming and camera status
    if (visionSubsystem.isCameraConnected()) {
      SmartDashboard.putString("Aiming Status", "Camera Connected");

      // Check if target has been specified
      // && (visionSubsystem.isTargetVisible(targetID))
      if ((targetID != AprilTags.None.getId()) && (visionSubsystem.isTargetVisible((int) targetID))) {
        // If the camera is connected, get the target yaw and drive towards it
        targetYaw = visionSubsystem.getTargetYaw((int) targetID);
        tagRange = visionSubsystem.getTargetRange((int) targetID);
        SmartDashboard.putNumber("Target ID", targetID);

        // Check if the target yaw is valid and displays the aiming status and yaw
        if (!Double.isNaN(targetYaw)) {
          // turn = ((desiredYaw - targetYaw) * visionConstantAngle *
          // AutoConstants.kMaxAngularSpeedRadiansPerSecond);
          turnAngularSpeed = (turnPIDController.calculate(targetYaw, desiredYaw));
          SmartDashboard.putString("Aiming Status", "Aiming");
          SmartDashboard.putNumber("Target Yaw", targetYaw);

        } else {
          SmartDashboard.putString("Aiming Status", "No Valid Target");
        }

        // Check if the target range is valid and update forward speed
        if (!Double.isNaN(tagRange)) {
          forwardLinearSpeed = (forwardPIDController.calculate(tagRange * Math.cos(Math.toRadians(targetYaw)),
              desiredYOffset));
          strafeLinearSpeed = (strafePIDController.calculate(tagRange * Math.sin(Math.toRadians(targetYaw)),
              desiredXOffset));
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

    forwardAdjustedLinearSpeed = x_rate.calculate(forwardLinearSpeed, linearAccelerationLimit);
    forwardAdjustedVoltage = forwardAdjustedLinearSpeed * DriveConstants.kForwardVoltsPerMeterPerSecond;
    forwardAdjustedPWM = forwardAdjustedVoltage / DriveConstants.kDefaultBusVoltage;

    strafeAdjustedLinearSpeed = y_rate.calculate(strafeLinearSpeed, linearAccelerationLimit);
    strafeAdjustedVoltage = strafeAdjustedLinearSpeed * DriveConstants.kStrafeVoltsPerMeterPerSecond;
    strafeAdjustedPWM = strafeAdjustedVoltage / DriveConstants.kDefaultBusVoltage;

    turnAdjustedAngularSpeed = theta_rate.calculate(turnAngularSpeed);
    turnAdjustedVoltage = turnAdjustedAngularSpeed * DriveConstants.kVoltsPerDegreePerSecond;
    turnAdjustedPWM = turnAdjustedVoltage / DriveConstants.kDefaultBusVoltage;

    forward = forwardAdjustedPWM;
    strafe = strafeAdjustedPWM;
    turn = turnAdjustedPWM;

    DriveTargetCommandLogger();
    // Drive robot with values calculated above
    // driveSubsystem.drive(forward, -strafe, turn, false);
  }

  // Ends the drivetarget command
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false);
    SmartDashboard.putString("Aiming Status", "Command Ended");
  }

  // Logistic Regression of a value to a value between [-1,1]
  public double SigmoidAdjustment(double value) {
    return (value) / (1 + Math.abs(value));
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
    SmartDashboard.putNumber("Turn", turn);
    SmartDashboard.putNumber("Turn PWM", turnPWM);
    SmartDashboard.putNumber("Turn Voltage", turnVoltage);
    SmartDashboard.putNumber("Turn Angular Speed", turnAngularSpeed);
    SmartDashboard.putNumber("Turn Adjusted Angular Speed", turnAdjustedAngularSpeed);
    SmartDashboard.putNumber("Turn Adjusted Voltage", turnAdjustedVoltage);
    SmartDashboard.putNumber("Turn Adjusted PWM", turnAdjustedPWM);
  }
}
