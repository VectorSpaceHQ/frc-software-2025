package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.DynamicSlewRateLimiter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RobotPoseEstimatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.AprilTags;
import org.photonvision.PhotonUtils;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class DriveTargetCommand extends Command {

  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private CommandXboxController driverController;
  private RobotPoseEstimatorSubsystem poseEstimator = null;
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
  private PIDController forwardPIDController = new PIDController(0.4, 0, 0);
  private PIDController strafePIDController = new PIDController(0.4, 0, 0);
  private PIDController turnPIDController = new PIDController(0.05, 0, 0);

  private final double desiredYaw = 0.0;
  private final double desiredYOffset = 0.3;
  private final double desiredXOffset = 0.0;
  
  // A hashmap that stores key (id) and value (target/range) pairs
  private Map<Integer, Double> lastValidYaw = new HashMap<>();
  private Map<Integer, Double> lastValidRange = new HashMap<>();

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

  // Set the pose estimator to use for targeting
  public void setPoseEstimator(RobotPoseEstimatorSubsystem estimator) {
    this.poseEstimator = estimator;
  }

  public void setTargetID(AprilTags tagId) {
    this.targetID = tagId.getId(); // Set targetID based on the provided AprilTags enum value
  }

  // Get the best available robot pose
  private Pose2d getBestRobotPose() {
    Pose2d robotPose = new Pose2d();
    
    if (poseEstimator != null) {
        robotPose = poseEstimator.getPose();
        SmartDashboard.putString("Pose Source", "Pose Estimator");
    } 
    else if (visionSubsystem.isFreshPose()) {
        Optional<Pose2d> visionPose = visionSubsystem.getRobotPose();
        if (visionPose.isPresent()) {
            robotPose = visionPose.get();
            SmartDashboard.putString("Pose Source", "Vision Direct");
        }
    }
    
    return robotPose;
  }

  // Calculate target yaw and range based on robot and target poses
  private boolean calculateTargetYawAndRange(int targetId) {
    boolean success = false;
    
    // Get robot pose from estimator or vision
    Pose2d robotPose = getBestRobotPose();
    
    // Get target pose
    Optional<Pose3d> tagPose3d = visionSubsystem.getTagPose(targetId);
    
    if (tagPose3d.isPresent()) {
        // Convert to 2D pose for calculations
        Pose2d tagPose2d = tagPose3d.get().toPose2d();
        
        // Calculate yaw and range using PhotonUtils
        Rotation2d yawRotation = PhotonUtils.getYawToPose(robotPose, tagPose2d);
        targetYaw = yawRotation.getDegrees();
        tagRange = PhotonUtils.getDistanceToPose(robotPose, tagPose2d);
        
        // Log to SmartDashboard
        SmartDashboard.putNumber("Raw Target Yaw", targetYaw);
        SmartDashboard.putNumber("Raw Target Range", tagRange);
        
        // Store valid measurements for future use
        if (!Double.isNaN(targetYaw)) {
            lastValidYaw.put(targetId, targetYaw);
        }
        
        if (!Double.isNaN(tagRange)) {
            lastValidRange.put(targetId, tagRange);
        }
        
        success = true;
    }
    
    // If current calculation failed but we have previous values, use those
    if (!success) {
        if (lastValidYaw.containsKey(targetId)) {
            targetYaw = lastValidYaw.get(targetId);
            SmartDashboard.putString("Yaw Status", "Using Last Valid");
        }
        
        if (lastValidRange.containsKey(targetId)) {
            tagRange = lastValidRange.get(targetId);
            SmartDashboard.putString("Range Status", "Using Last Valid");
        }
        
        success = lastValidYaw.containsKey(targetId) || lastValidRange.containsKey(targetId);
    }
    
    return success;
  }

  // Executes the drivetarget command (periodic)
  @Override
  public void execute() {
    linearAccelerationLimit = table.get(elevatorSubsystem.getElevatorHeight());

    forwardPWM = driverController.getLeftY() * speedscalar;
    forwardVoltage = forwardPWM * DriveConstants.kDefaultBusVoltage;
    forwardLinearSpeed = forwardVoltage / DriveConstants.kForwardVoltsPerMeterPerSecond;

    strafePWM = driverController.getLeftX() * speedscalar;
    strafeVoltage = strafePWM * DriveConstants.kDefaultBusVoltage;
    strafeLinearSpeed = strafeVoltage / DriveConstants.kStrafeVoltsPerMeterPerSecond;

    turnPWM = -0.3 * driverController.getRightX() * AutoConstants.kMaxAngularSpeedRadiansPerSecond;
    turnVoltage = turnPWM * DriveConstants.kDefaultBusVoltage;
    turnAngularSpeed = turnVoltage / DriveConstants.kVoltsPerDegreePerSecond;

    // Check if the camera is connected and displays the aiming and camera status
    if (visionSubsystem.isCameraConnected()) {
      SmartDashboard.putString("Aiming Status", "Camera Connected");

      // Check if target has been specified
      if ((targetID != AprilTags.None.getId())) {
        // Calculate yaw and range to target
        boolean targetFound = calculateTargetYawAndRange((int)targetID);
        
        if (targetFound) {
          SmartDashboard.putNumber("Target ID", targetID);
          
          // Use yaw for turning
          if (!Double.isNaN(targetYaw)) {
            turnAngularSpeed = turnPIDController.calculate(targetYaw, desiredYaw);
            SmartDashboard.putString("Aiming Status", "Aiming");
            SmartDashboard.putNumber("Target Yaw", targetYaw);
          }
          
          // Use range for forward/strafe movement
          if (!Double.isNaN(tagRange)) {
            forwardLinearSpeed = forwardPIDController.calculate(
                tagRange * Math.cos(Math.toRadians(targetYaw)), desiredYOffset);
            
            strafeLinearSpeed = strafePIDController.calculate(
                tagRange * Math.sin(Math.toRadians(targetYaw)), desiredXOffset);
            
            SmartDashboard.putString("Aiming Status", "Driving Forward");
            SmartDashboard.putNumber("Target Range", tagRange);
          }
          
          if (!Double.isNaN(targetYaw) && !Double.isNaN(tagRange)) {
            SmartDashboard.putString("Aiming Status", "Aiming and Driving Forward");
          }
        } else {
          SmartDashboard.putString("Aiming Status", "No Valid Target");
        }
      } else {
        SmartDashboard.putString("Aiming Status", "No Target Specified");
      }
    } else {
      SmartDashboard.putString("Aiming Status", "Camera Not Connected");
    }

    // Apply rate limiting to all motion
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