package frc.robot.commands;

// Imports for the drivetarget command
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DriveConstants;
import frc.robot.DynamicSlewRateLimiter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RobotPoseEstimatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.AprilTags;

public class DriveTargetCommand extends Command {

  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private CommandXboxController driverController;
  private RobotPoseEstimatorSubsystem poseEstimator;


  private double speedscalar = 1;
  private double forward =  0;
  private double strafe =  0;
  private double turn = 0;

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
  private InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
  private double targetID = AprilTags.None.getId(); // Initialize targetID to None

  private final ShuffleboardTab targetingTab = Shuffleboard.getTab("Targeting");
  private final ShuffleboardLayout statusCol = targetingTab.getLayout("Status", BuiltInLayouts.kList).withPosition(0,0).withSize(1,3);
  private final ShuffleboardLayout forwardCol = targetingTab.getLayout("Forward", BuiltInLayouts.kList).withPosition(1,0).withSize(2,5);
  private final ShuffleboardLayout strafeCol = targetingTab.getLayout("Strafe", BuiltInLayouts.kList).withPosition(3,0).withSize(2,5);
  private final GenericEntry aimingStatusEntry = statusCol.add("Aiming Status", "Idle").getEntry();
  private final GenericEntry linearAccelerationLimitEntry = statusCol.add("Linear Accel Limit", 0.0).getEntry();
  private final GenericEntry forwardEntry = forwardCol.add("Fwd Output", 0.0).getEntry();
  private final GenericEntry forwardPwmEntry = forwardCol.add("Fwd PWM", 0.0).getEntry();
  private final GenericEntry forwardVoltageEntry = forwardCol.add("Fwd Voltage", 0.0).getEntry();
  private final GenericEntry forwardLinearSpeedEntry = forwardCol.add("Fwd Linear Speed", 0.0).getEntry();
  private final GenericEntry forwardAdjustedLinearSpeedEntry = forwardCol.add("Fwd Adj Linear Speed", 0.0).getEntry();
  private final GenericEntry forwardAdjustedVoltageEntry = forwardCol.add("Fwd Adj Voltage", 0.0).getEntry();
  private final GenericEntry forwardAdjustedPwmEntry = forwardCol.add("Fwd Adj PWM", 0.0).getEntry();
  private final GenericEntry strafeEntry = strafeCol.add("Strafe Output", 0.0).getEntry();
  private final GenericEntry strafePwmEntry = strafeCol.add("Strafe PWM", 0.0).getEntry();
  private final GenericEntry strafeVoltageEntry = strafeCol.add("Strafe Voltage", 0.0).getEntry();
  private final GenericEntry strafeLinearSpeedEntry = strafeCol.add("Strafe Linear Speed", 0.0).getEntry();
  private final GenericEntry strafeAdjustedLinearSpeedEntry = strafeCol.add("Strafe Adj Linear Speed", 0.0).getEntry();
  private final GenericEntry strafeAdjustedVoltageEntry = strafeCol.add("Strafe Adj Voltage", 0.0).getEntry();
  private final GenericEntry strafeAdjustedPwmEntry = strafeCol.add("Strafe Adj PWM", 0.0).getEntry();

  public void setTargetID(AprilTags tagId) {
    this.targetID = tagId.getId(); // Set targetID based on the provided AprilTags enum value
  }

  // Sets the drivetarget constructor
  public DriveTargetCommand(
      DriveSubsystem driveSubsystem,
      CommandXboxController driverController,
      ElevatorSubsystem elevatorSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.driverController = driverController;
    this.elevatorSubsystem = elevatorSubsystem;
    table.put(elevatorSubsystem.getMinHeight(), DriveConstants.kMaxAcceleration);
    table.put(elevatorSubsystem.getMaxHeight(), DriveConstants.kMinAcceleration);
    addRequirements(driveSubsystem);
    setTargetID(AprilTags.None);
  }
  public void setPoseEstimator(RobotPoseEstimatorSubsystem estimator) {
    this.poseEstimator = estimator;
  }

  // Executes the drivetarget command (periodic)
  @Override
  public void execute() {

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
    turn = (-0.3 * driverController.getRightX());
    DriveTargetCommandLogger();
    driveSubsystem.drive(forward, strafe, turn, true);
  }

  // Ends the drivetarget command
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, true);
    aimingStatusEntry.setString("Command Ended");
  }

  public void setSpeedScalar(double val){
    speedscalar = val;
  }

  private void DriveTargetCommandLogger() {
    linearAccelerationLimitEntry.setDouble(linearAccelerationLimit);
    forwardEntry.setDouble(forward);
    forwardPwmEntry.setDouble(forwardPWM);
    forwardVoltageEntry.setDouble(forwardVoltage);
    forwardLinearSpeedEntry.setDouble(forwardLinearSpeed);
    forwardAdjustedLinearSpeedEntry.setDouble(forwardAdjustedLinearSpeed);
    forwardAdjustedVoltageEntry.setDouble(forwardAdjustedVoltage);
    forwardAdjustedPwmEntry.setDouble(forwardAdjustedPWM);
    strafeEntry.setDouble(strafe);
    strafePwmEntry.setDouble(strafePWM);
    strafeVoltageEntry.setDouble(strafeVoltage);
    strafeLinearSpeedEntry.setDouble(strafeLinearSpeed);
    strafeAdjustedLinearSpeedEntry.setDouble(strafeAdjustedLinearSpeed);
    strafeAdjustedVoltageEntry.setDouble(strafeAdjustedVoltage);
    strafeAdjustedPwmEntry.setDouble(strafeAdjustedPWM);
    aimingStatusEntry.setString("Running");
  }
}






